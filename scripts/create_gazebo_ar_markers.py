#!/usr/bin/env python

## A script that generates AR markers that can be used in Gazebo.
# @ingroup utilities
# @file create_gazebo_ar_markers.py

# Copyright (c) 2021, Robot Control and Pattern Recognition Group,
# Institute of Control and Computation Engineering
# Warsaw University of Technology
#
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Warsaw University of Technology nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYright HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Author: Dawid Seredynski
#

import roslib; roslib.load_manifest('rcprg_gazebo_utils')

import sys
import rospy
import math
import copy
import tf

from std_msgs.msg import ColorRGBA
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
from geometry_msgs.msg import *
from tf.transformations import * 
import tf_conversions.posemath as pm
import PyKDL
from cartesian_trajectory_msgs.msg import *
import actionlib
from gazebo_msgs.srv import *


import os
import subprocess

def getMarkerModelName(size_cm, marker_id):
    return 'ar_marker_{}cm_{}'.format(size_cm, marker_id)

def getMeshFilename(size_cm):
    return 'ar_marker_{}cm.dae'.format(size_cm)

def getMaterialFilename(marker_id):
    return 'MarkerData_{}.png'.format(marker_id)

def generateModelConfig(size_cm, marker_id):
    return '<?xml version="1.0"?>\n' +\
        '<model>\n' +\
        '  <name>AR Marker {}cm {}</name>\n'.format(size_cm, marker_id) +\
        '  <version>1.0</version>\n' +\
        '  <sdf version="1.5">model.sdf</sdf>\n' +\
        '  <author>\n' +\
        '    <name>Dawid Seredynski</name>\n' +\
        '    <email>dawid.seredynski(at)gmail.com</email>\n' +\
        '  </author>\n' +\
        '  <description>\n' +\
        '    AR marker {}cm {}.\n'.format(size_cm, marker_id) +\
        '  </description>\n' +\
        '</model>\n'

def generateSdf(size_cm, marker_id):
    return '<?xml version="1.0" ?>\n' +\
        '<sdf version="1.5">\n' +\
        '  <model name="{}">\n'.format( getMarkerModelName(size_cm, marker_id) ) +\
        '    <link name="link">\n' +\
        '      <gravity>false</gravity>\n' +\
        '      <inertial>\n' +\
        '        <pose>0 0 0 0 0 0</pose>\n' +\
        '        <mass>0.01</mass>\n' +\
        '        <inertia>\n' +\
        '          <ixx>0.000079</ixx>\n' +\
        '          <ixy>0</ixy>\n' +\
        '          <ixz>0</ixz>\n' +\
        '          <iyy>0.000079</iyy>\n' +\
        '          <iyz>0</iyz>\n' +\
        '          <izz>0.000025</izz>\n' +\
        '        </inertia>\n' +\
        '      </inertial>\n' +\
        '      <visual name="visual">\n' +\
        '        <pose>0 0 0 0 0 0</pose>\n' +\
        '        <geometry>\n' +\
        '          <mesh>\n' +\
        '            <uri>model://{}/meshes/{}</uri>\n'.format(
                                getMarkerModelName(size_cm, marker_id), getMeshFilename(size_cm)) +\
        '          </mesh>\n' +\
        '        </geometry>\n' +\
        '      </visual>\n' +\
        '    </link>\n' +\
        '  </model>\n' +\
        '</sdf>\n'

def generateMesh(size_cm, marker_id):
    return '<?xml version="1.0" encoding="utf-8"?>\n' +\
        '<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance">\n' +\
        '  <asset>\n' +\
        '    <contributor>\n' +\
        '      <author>Blender User</author>\n' +\
        '      <authoring_tool>Blender 2.91.0 commit date:2020-11-25, commit time:08:34, hash:0f45cab862b8</authoring_tool>\n' +\
        '    </contributor>\n' +\
        '    <created>2021-05-19T08:11:01</created>\n' +\
        '    <modified>2021-05-19T08:11:01</modified>\n' +\
        '    <unit name="meter" meter="1"/>\n' +\
        '    <up_axis>Z_UP</up_axis>\n' +\
        '  </asset>\n' +\
        '  <library_effects>\n' +\
        '    <effect id="Material_002-effect">\n' +\
        '      <profile_COMMON>\n' +\
        '        <newparam sid="markerdata_png-surface">\n' +\
        '          <surface type="2D">\n' +\
        '            <init_from>markerdata_png</init_from>\n' +\
        '          </surface>\n' +\
        '        </newparam>\n' +\
        '        <newparam sid="markerdata_png-sampler">\n' +\
        '          <sampler2D>\n' +\
        '            <source>markerdata_png-surface</source>\n' +\
        '          </sampler2D>\n' +\
        '        </newparam>\n' +\
        '        <technique sid="common">\n' +\
        '          <lambert>\n' +\
        '            <emission>\n' +\
        '              <color sid="emission">0 0 0 1</color>\n' +\
        '            </emission>\n' +\
        '            <diffuse>\n' +\
        '              <texture texture="markerdata_png-sampler" texcoord="Cylinder-mesh-map-0"/>\n' +\
        '            </diffuse>\n' +\
        '            <index_of_refraction>\n' +\
        '              <float sid="ior">1</float>\n' +\
        '            </index_of_refraction>\n' +\
        '          </lambert>\n' +\
        '        </technique>\n' +\
        '      </profile_COMMON>\n' +\
        '    </effect>\n' +\
        '    <effect id="Material_001-effect">\n' +\
        '      <profile_COMMON>\n' +\
        '        <technique sid="common">\n' +\
        '          <lambert>\n' +\
        '            <emission>\n' +\
        '              <color sid="emission">0 0 0 1</color>\n' +\
        '            </emission>\n' +\
        '            <diffuse>\n' +\
        '              <color sid="diffuse">0.512 0.512 0.512 1</color>\n' +\
        '            </diffuse>\n' +\
        '            <index_of_refraction>\n' +\
        '              <float sid="ior">1</float>\n' +\
        '            </index_of_refraction>\n' +\
        '          </lambert>\n' +\
        '        </technique>\n' +\
        '      </profile_COMMON>\n' +\
        '    </effect>\n' +\
        '  </library_effects>\n' +\
        '  <library_images>\n' +\
        '    <image id="markerdata_png" name="markerdata_png">\n' +\
        '      <init_from>../materials/{}</init_from>\n'.format( getMaterialFilename(marker_id) ) +\
        '    </image>\n' +\
        '  </library_images>\n' +\
        '  <library_materials>\n' +\
        '    <material id="Material_002-material" name="Material_002">\n' +\
        '      <instance_effect url="#Material_002-effect"/>\n' +\
        '    </material>\n' +\
        '    <material id="Material_001-material" name="Material_001">\n' +\
        '      <instance_effect url="#Material_001-effect"/>\n' +\
        '    </material>\n' +\
        '  </library_materials>\n' +\
        '  <library_geometries>\n' +\
        '    <geometry id="Cylinder-mesh" name="Cylinder">\n' +\
        '      <mesh>\n' +\
        '        <source id="Cylinder-mesh-positions">\n' +\
        '          <float_array id="Cylinder-mesh-positions-array" count="24">-0.5 -0.5 -0.5 -0.5 0.5 -0.5 0.5 0.5 -0.5 0.5 -0.5 -0.5 -0.5 -0.5 0.5 -0.5 0.5 0.5 0.5 0.5 0.5 0.5 -0.5 0.5</float_array>\n' +\
        '          <technique_common>\n' +\
        '            <accessor source="#Cylinder-mesh-positions-array" count="8" stride="3">\n' +\
        '              <param name="X" type="float"/>\n' +\
        '              <param name="Y" type="float"/>\n' +\
        '              <param name="Z" type="float"/>\n' +\
        '            </accessor>\n' +\
        '          </technique_common>\n' +\
        '        </source>\n' +\
        '        <source id="Cylinder-mesh-normals">\n' +\
        '          <float_array id="Cylinder-mesh-normals-array" count="18">-1 0 0 0 1 0 1 0 0 0 -1 0 0 0 -1 0 0 1</float_array>\n' +\
        '          <technique_common>\n' +\
        '            <accessor source="#Cylinder-mesh-normals-array" count="6" stride="3">\n' +\
        '              <param name="X" type="float"/>\n' +\
        '              <param name="Y" type="float"/>\n' +\
        '              <param name="Z" type="float"/>\n' +\
        '            </accessor>\n' +\
        '          </technique_common>\n' +\
        '        </source>\n' +\
        '        <source id="Cylinder-mesh-map-0">\n' +\
        '          <float_array id="Cylinder-mesh-map-0-array" count="72">0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0.9998999 9.998e-5 0.9999001 0.9998999 1.00099e-4 0.9998999 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 9.998e-5 1.0001e-4 0.9998999 9.998e-5 1.00099e-4 0.9998999</float_array>\n' +\
        '          <technique_common>\n' +\
        '            <accessor source="#Cylinder-mesh-map-0-array" count="36" stride="2">\n' +\
        '              <param name="S" type="float"/>\n' +\
        '              <param name="T" type="float"/>\n' +\
        '            </accessor>\n' +\
        '          </technique_common>\n' +\
        '        </source>\n' +\
        '        <vertices id="Cylinder-mesh-vertices">\n' +\
        '          <input semantic="POSITION" source="#Cylinder-mesh-positions"/>\n' +\
        '        </vertices>\n' +\
        '        <triangles material="Material_002-material" count="12">\n' +\
        '          <input semantic="VERTEX" source="#Cylinder-mesh-vertices" offset="0"/>\n' +\
        '          <input semantic="NORMAL" source="#Cylinder-mesh-normals" offset="1"/>\n' +\
        '          <input semantic="TEXCOORD" source="#Cylinder-mesh-map-0" offset="2" set="0"/>\n' +\
        '          <p>4 0 0 5 0 1 1 0 2 5 1 3 6 1 4 2 1 5 6 2 6 7 2 7 3 2 8 7 3 9 4 3 10 0 3 11 0 4 12 1 4 13 2 4 14 7 5 15 6 5 16 5 5 17 0 0 18 4 0 19 1 0 20 1 1 21 5 1 22 2 1 23 2 2 24 6 2 25 3 2 26 3 3 27 7 3 28 0 3 29 3 4 30 0 4 31 2 4 32 4 5 33 7 5 34 5 5 35</p>\n' +\
        '        </triangles>\n' +\
        '      </mesh>\n' +\
        '    </geometry>\n' +\
        '  </library_geometries>\n' +\
        '  <library_visual_scenes>\n' +\
        '    <visual_scene id="Scene" name="Scene">\n' +\
        '      <node id="Cylinder" name="Cylinder" type="NODE">\n' +\
        '        <matrix sid="transform">{} 0 0 0 0 {} 0 0 0 0 {} 0 0 0 0 1</matrix>\n'.format(0.01*size_cm, 0.01*size_cm, 0.002) +\
        '        <instance_geometry url="#Cylinder-mesh" name="Cylinder">\n' +\
        '          <bind_material>\n' +\
        '            <technique_common>\n' +\
        '              <instance_material symbol="Material_002-material" target="#Material_002-material">\n' +\
        '                <bind_vertex_input semantic="Cylinder-mesh-map-0" input_semantic="TEXCOORD" input_set="0"/>\n' +\
        '              </instance_material>\n' +\
        '              <instance_material symbol="Material_001-material" target="#Material_001-material">\n' +\
        '                <bind_vertex_input semantic="Cylinder-mesh-map-0" input_semantic="TEXCOORD" input_set="0"/>\n' +\
        '              </instance_material>\n' +\
        '            </technique_common>\n' +\
        '          </bind_material>\n' +\
        '        </instance_geometry>\n' +\
        '      </node>\n' +\
        '    </visual_scene>\n' +\
        '  </library_visual_scenes>\n' +\
        '  <scene>\n' +\
        '    <instance_visual_scene url="#Scene"/>\n' +\
        '  </scene>\n' +\
        '</COLLADA>'

def createMaterialFile(path, size_cm, marker_id):
    subprocess.call(['rosrun', 'ar_track_alvar', 'createMarker', '-s', str(size_cm), str(marker_id)],
        cwd=path)

def createMarkerGazeboModel(path, size_cm, marker_id):
    dir_name = getMarkerModelName(size_cm, marker_id)
    path_in = path + '/' + dir_name

    try:
        os.mkdir(path_in)
    except OSError:
        print ("Creation of the directory %s failed" % path)
        return False
    else:
        print ("Successfully created the directory %s " % path)

    with open(path_in + '/model.config', 'w') as f:
        f.write( generateModelConfig(size_cm, marker_id) )

    with open(path_in + '/model.sdf', 'w') as f:
        f.write( generateSdf(size_cm, marker_id) )

    path_meshes = path_in + '/meshes'
    os.mkdir(path_meshes)
    with open(path_meshes + '/' + getMeshFilename(size_cm), 'w') as f:
        f.write( generateMesh(size_cm, marker_id) )

    path_materials = path_in + '/materials'
    os.mkdir(path_materials)
    createMaterialFile(path_materials, size_cm, marker_id)

if __name__ == "__main__":

    for marker_id in range(0, 35):
        createMarkerGazeboModel('.', 8, marker_id)
    exit(0)
