# Copyright 2024 R(obots) V(ision) and P(erception) group
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors
#    may be used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import os
import sys
from pathlib import Path
from typing import Tuple
import numpy as np

from mad_icp.apps.utils.point_cloud2 import read_point_cloud

# ⬇️ ADDED
from pyproj import Transformer
from collections import deque

from rosbags.serde import deserialize_cdr
from rosbags.typesys import Stores, get_types_from_msg, get_typestore

from rosbags.typesys import Stores, get_typestore
from rosbags.typesys.msg import get_types_from_msg
from rosbags.serde import deserialize_cdr

import struct

# Full definition of gps_msgs/msg/GPSFix and its dependencies
GPS_FIX_FULL_DEF = {
    'gps_msgs/msg/GPSFix': """
        std_msgs/Header header
        float64 latitude
        float64 longitude
        float64 altitude
        float64 track
        float64 speed
        float64 climb
        float64 pitch
        float64 roll
        float64 dip
        float64 time
        float64 gdop
        float64 pdop
        float64 hdop
        float64 vdop
        float64 tdop
        float64 err
        float64 err_horz
        float64 err_vert
        float64 err_track
        float64 err_speed
        float64 err_climb
        float64 err_time
        uint16 satellites_used
        """,
            'std_msgs/msg/Header': """
        builtin_interfaces/Time stamp
        string frame_id
        """,
            'builtin_interfaces/msg/Time': """
        int32 sec
        uint32 nanosec
    """
}


typestore = get_typestore(Stores.ROS2_FOXY)
for msg_type, definition in GPS_FIX_FULL_DEF.items():
    typestore.register(get_types_from_msg(definition, msg_type))


# typestore = get_typestore('cdr')
# typestore.register('gps_msgs/msg/GPSFix', *parse_definition(GPS_FIX_DEF, 'gps_msgs/msg/GPSFix'))

class Ros2Reader:
    def __init__(self, data_dir: Path, min_range=0, max_range=200, *args, **kwargs):
        topic = kwargs.pop('topic')
        self.min_range = min_range
        self.max_range = max_range
        self.topic = topic

        try:
            from rosbags.highlevel import AnyReader
        except ModuleNotFoundError:
            print("Rosbags library not installed, run 'pip install -U rosbags'")
            sys.exit(-1)

        self.bag = AnyReader([data_dir])
        # path to your custom .msg file
        msg_path = os.path.join(os.path.dirname(__file__), "rosbags_custom_types")

        # register the GPSFix message type with rosbags
        # register_msgdef(GPS_FIX_DEF , 'gps_msgs/msg/GPSFix')
        self.bag.typestore = typestore
        self.bag.open()

        if not topic:
            raise Exception("You have to specify a topic")

        print("Reading the following topic: ", topic)

        self.pc_connections = [x for x in self.bag.connections if x.topic == topic]
        self.pc_msgs = self.bag.messages(connections=self.pc_connections)
        self.num_messages = self.bag.topics[topic].msgcount

        # ⬇️ ADDED: prepare GPS message list
        self.gps_connections = [x for x in self.bag.connections if x.topic == '/gps']
        self.gps_msgs = list(self.bag.messages(connections=self.gps_connections))
        
        # print("DEBUG")
        # print(self.gps_msgs)

        # ⬇️ ADDED: preload gps fixes and transformer
        self.gps_data = []
        self._load_gps()
        self.transformer = Transformer.from_crs("EPSG:4326", "EPSG:4978", always_xy=True)
        self.ref_ecef = self._gps_to_ecef(*self.gps_data[0][1:])  # (x, y, z)

    def __len__(self):
        return self.num_messages

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        if hasattr(self, "bag"):
            self.bag.close()

    def _load_gps(self):
        for connection, timestamp, rawdata in self.gps_msgs:
            # skip 4-byte CDR header
            cdr = rawdata[4:]

            offset = 0

            # 1. Read Header.stamp (int32  sec + uint32 nanosec)
            sec, nanosec = struct.unpack_from('<iI', cdr, offset)
            offset += 8

            # 2. Read string length of frame_id
            strlen = struct.unpack_from('<I', cdr, offset)[0]
            offset += 4 + strlen
            offset = (offset + 3) & ~3  # 4-byte align

            # 3. Read latitude, longitude, altitude (3 float64 = 24 bytes)
            latitude, longitude, altitude = struct.unpack_from('<ddd', cdr, offset)

            stamp = sec + nanosec * 1e-9
            self.gps_data.append((stamp, longitude, latitude, altitude))

        
    def _gps_to_ecef(self, lon, lat, alt):
        return np.array(self.transformer.transform(lon, lat, alt))

    def _interpolate_gps(self, ts):
        if ts <= self.gps_data[0][0]:
            return self.gps_data[0][1:]
        if ts >= self.gps_data[-1][0]:
            return self.gps_data[-1][1:]

        for i in range(1, len(self.gps_data)):
            t0, lon0, lat0, alt0 = self.gps_data[i - 1]
            t1, lon1, lat1, alt1 = self.gps_data[i]
            if t0 <= ts <= t1:
                r = (ts - t0) / (t1 - t0)
                lon = lon0 + r * (lon1 - lon0)
                lat = lat0 + r * (lat1 - lat0)
                alt = alt0 + r * (alt1 - alt0)
                return lon, lat, alt
        return self.gps_data[-1][1:]

    def __getitem__(self, item) -> Tuple[float, np.ndarray]:
        connection, timestamp, rawdata = next(self.pc_msgs)
        msg = self.bag.deserialize(rawdata, connection.msgtype)
        cloud_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        points, _ = read_point_cloud(msg, min_range=self.min_range, max_range=self.max_range)

        # ⬇️ ADDED: get interpolated GPS fix
        lon, lat, alt = self._interpolate_gps(cloud_stamp)
        ecef = self._gps_to_ecef(lon, lat, alt)
        offset = ecef - self.ref_ecef  # ENU approximation

        # ⬇️ ADDED: inject gps as constant columns
        n_points = points.shape[0]
        gps_info = np.tile([lat, lon, alt], (n_points, 1))
        points_xyz_offset = points + offset  # relative movement
        enriched_points = np.concatenate([points_xyz_offset, gps_info], axis=1)

        return cloud_stamp, enriched_points

