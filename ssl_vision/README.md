# TrikABCL

[![Join the chat at https://gitter.im/TrikABCL/Lobby](https://badges.gitter.im/TrikABCL/Lobby.svg)](https://gitter.im/TrikABCL/Lobby?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge)

These are sslServer, staticObjectsServer and sslClient ROS nodes.
 
SslServer node runs on vision system server and publishes data sent by ssl-vision.
 
StaticObjectsServer node also runs on vision system server. It publishes coordinates of static objects, sent by ssl-vision's mark_static_objects plugin.

SslClient node runs on TRIK or PC. It decodes protobuf and displays messages published by ssl_talker.

