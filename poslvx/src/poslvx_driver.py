#!/usr/bin/env python

import rospy
import socket
from struct import unpack_from

from poslvx.msg import INS, INSRMS


def parse(data):
  STX = data[0]
  if STX != 0x02:
    rospy.logerr("Invalid start transmission byte: %s" % hex(STX))
    return

  status = data[1]
  rospy.logdebug("Status:", hex(status))

  packet_type = data[2]
  if packet_type != 0x40:
    rospy.logerr("Invalid packet type: ", packet_type)
    return
  else:
    rospy.logdebug("Packet type: ", hex(packet_type))

  length = data[3]
  rospy.logdebug("Length: ", length)

  transmission_number = data[4]
  rospy.logdebug("Transmission number: ", hex(transmission_number))

  page_index = data[5]
  max_page_index = data[6]
  if max_page_index != 0 or page_index != 0:
    rospy.logerr("page index larger than 0. Not support yet. page_index/max_page_index=%d/%d" % (page_index, max_page_index))
    return

  gsof_data = data[7:-2]

  checksum = data[-2]
  cs = (status + packet_type + length + sum(data[4:-2])) % 256
  if cs != checksum:
    rospy.logerr("Wrong checksum: %s vs %s" % (hex(cs), hex(checksum)))
    return

  EXT = data[-1]
  if EXT != 0x03:
    rospy.logerr("Invalid end transmission byte: ", hex(STX))
    return

  # GSOF
  try:
    while len(gsof_data) != 0:
      output_record_type = gsof_data[0]
      record_length = gsof_data[1]
      rospy.logdebug("Output record type:", hex(output_record_type), ", length of record: ", record_length)
      record = gsof_data[:(2 + record_length)]
      parse_gsof(output_record_type, gsof_data)
      gsof_data = gsof_data[(2 + record_length):]
    rospy.logdebug("All GSOF parsed")
  except Exception, e:
    rospy.logerr("Something wrong:")
    print(e)
    return

def parse_gsof(output_record_type, data):
  if output_record_type == 0x31:
    parse_gsof_49(data)
  elif output_record_type == 0x32:
    parse_gsof_50(data)
  elif output_record_type == 0x33:
    parse_gsof_51(data)
  else:
    rospy.logerr("Unknown output record type")
  
def parse_gsof_49(data):
  rospy.logdebug("Parsing GSOF 49")
  gps_week, gps_time = unpack_from(">hL", data, 2)
  imu_alignment_status = data[8]
  gnss_status = data[9]
  latitude, longitude, altitude = unpack_from(">ddd", data, 10)
  north_velocity, east_velocity, down_velocity, total_speed = unpack_from(">ffff", data, 34)
  roll, pitch, heading, track_angle = unpack_from(">dddd", data, 50)
  angular_rate_x, angular_rate_y, angular_rate_z = unpack_from(">fff", data, 82)
  acceleration_x, acceleration_y, acceleration_z = unpack_from(">fff", data, 94)

  msg = INS()
  msg.header.stamp = rospy.Time.now()
  msg.status.gps_week = gps_week
  msg.status.gps_time = gps_time
  msg.status.imu_alignment_status = imu_alignment_status
  msg.status.gnss_status = gnss_status
  msg.latitude = latitude
  msg.longitude = longitude
  msg.altitude = altitude
  msg.north_velocity = north_velocity
  msg.east_velocity = east_velocity
  msg.down_velocity = down_velocity
  msg.roll = roll
  msg.pitch = pitch
  msg.heading = heading
  msg.track_angle = track_angle
  msg.angular_rate_x = angular_rate_x
  msg.angular_rate_y = angular_rate_y
  msg.angular_rate_z = angular_rate_z
  msg.acceleration_x = acceleration_x
  msg.acceleration_y = acceleration_y
  msg.acceleration_z = acceleration_z
  msg.total_speed = total_speed
  data_pub.publish(msg)

  rospy.logdebug("GPS week: ", gps_week)
  rospy.logdebug("GPS time: ", gps_time)
  rospy.logdebug("imu_alignment_status: ", hex(imu_alignment_status))
  rospy.logdebug("gnss_status: ", hex(gnss_status))
  rospy.logdebug("latitude:", latitude, ", longitude:", longitude, ", altitude: ", altitude)
  rospy.logdebug("north_velocity: ", north_velocity, "east_velocity: ", east_velocity, "down_velocity: ", down_velocity, "total_speed: ", total_speed)
  rospy.logdebug("roll: ", roll, "pitch: ", pitch, "heading: ", heading, "track_angle: ", track_angle)
  rospy.logdebug("angular_rate_x: ", angular_rate_x, "angular_rate_y: ", angular_rate_y, "angular_rate_z: ", angular_rate_z)
  rospy.logdebug("acceleration_x: ", acceleration_x, "acceleration_y: ", acceleration_y, "acceleration_z: ", acceleration_z)


def parse_gsof_50(data):
  rospy.logdebug("Parsing GSOF 50")
  gps_week, gps_time = unpack_from(">hL", data, 2)
  imu_alignment_status = data[8]
  gnss_status = data[9]
  north_position_rms, east_position_rms, down_position_rms = unpack_from(">fff", data, 10)
  north_velocity_rms, east_velocity_rms, down_velocity_rms = unpack_from(">fff", data, 22)
  roll_rms, pitch_rms, heading_rms = unpack_from(">fff", data, 34)

  msg = INSRMS()
  msg.header.stamp = rospy.Time.now()
  msg.status.gps_week = gps_week
  msg.status.gps_time = gps_time
  msg.status.imu_alignment_status = imu_alignment_status
  msg.status.gnss_status = gnss_status
  msg.north_position_rms = north_position_rms
  msg.east_position_rms = east_position_rms
  msg.down_position_rms = down_position_rms
  msg.north_velocity_rms = north_velocity_rms
  msg.east_velocity_rms = east_velocity_rms
  msg.down_velocity_rms = down_velocity_rms
  msg.roll_rms = roll_rms
  msg.pitch_rms = pitch_rms
  msg.heading_rms = heading_rms
  rms_pub.publish(msg)

  rospy.logdebug("GPS week: ", gps_week)
  rospy.logdebug("GPS time: ", gps_time)
  rospy.logdebug("imu_alignment_status: ", hex(imu_alignment_status))
  rospy.logdebug("gnss_status: ", hex(gnss_status))
  rospy.logdebug("north_position_rms: ", north_position_rms, "east_position_rms: ", east_position_rms, "down_position_rms: ", down_position_rms)
  rospy.logdebug("north_velocity_rms: ", north_velocity_rms, "east_velocity_rms: ", east_velocity_rms, "down_velocity_rms: ", down_velocity_rms)
  rospy.logdebug("roll_rms: ", roll_rms, "pitch_rms: ", pitch_rms, "heading_rms: ", heading_rms)


rospy.init_node("poslvx")

data_pub = rospy.Publisher("ins/data", INS, queue_size=100)
rms_pub = rospy.Publisher("ins/rms", INSRMS, queue_size=100)

ADDRESS = ('192.168.53.100', 5017)
MAX_BYTES = 512

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect(ADDRESS)

while not rospy.is_shutdown():
  data = s.recv(MAX_BYTES)
  rospy.logdebug("len(data) is ", len(data))
  data = bytearray(data)
  parse(data)
s.close()
