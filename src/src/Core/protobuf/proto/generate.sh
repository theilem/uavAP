#!/bin/bash
cd ../../../../
rm uavAP/Core/protobuf/messages/*.pb.h
rm src/Core/protobuf/messages/*.pb.cc
protoc --cpp_out=uavAP/Core/protobuf/messages/ -Isrc/Core/protobuf/proto/ src/Core/protobuf/proto/*.proto 
cd uavAP/Core/protobuf/messages/
mv $(ls | grep .pb.cc) ../../../../src/Core/protobuf/messages
