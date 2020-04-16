#FOR FETCH
from lxml import etree
import numpy as np
import os
from dm_control import mjcf
import os
from dm_control.mujoco.wrapper import util
import six
from IPython import embed
import pdb
from xml.dom import minidom

target_size = "0.02 0.02 0.02"
colors = { 
	0 : '0.5 0.5 0.8 1', 
    1 : '1 1 0 1', 
    2 : '1 0.5 0.5 1', 
    3 : '0.5 1 1 1',  
    4 : '0.8 0.8 1 1' 
    }
total_num = 1
object_size = "0.025 0.025 0.025"
gRange = 0.25


def get_pos():
	pos = np.random.uniform(low=-gRange, high= gRange, size = 2)
	return '%.2f %.2f' % (pos[0], pos[1])

def get_color():
	color = np.random.uniform(low=0, high=1, size=3)
	rgba = "%.2f %.2f %.2f 1" % (color[0], color[1], color[2])
	return rgba

def make_object(body_id, mjcf_model, **kwargs):
	"""
	<body name="object0" pos="0.025 0.025 0.025">
	<joint name="object0:joint" type="free" damping="0.01"></joint>
	<geom size="0.025 0.025 0.025" type="box" condim="3" name="object0" material="block_mat" mass="2"></geom>
	<site name="object0" pos="0 0 0" size="0.02 0.02 0.02" rgba="1 0 0 1" type="sphere"></site>
	</body>
	"""

	global total_num
	body_name = "object{}".format(body_id)
	joint_name0 = 'object{}_x'.format(body_id)
	joint_name1 = 'object{}_y'.format(body_id)
	geom_name = 'object{}'.format(body_id)

	body = etree.Element('body', name=body_name)
	pos = get_pos()
	pos = pos + " 0.025"
	body.set('pos', pos)
	joint = etree.SubElement(body, 'joint', name = 'object{}:joint'.format(body_id), type="free")
	# color = get_color()

	if total_num < 6:
		color = colors[total_num]
		total_num += 1
	
	geom = etree.SubElement(body, 
		'geom', 
		name=geom_name, 
		type='box', 
		size=object_size, 
		rgba=color, 
		material='block_mat',
		condim="3",
		mass="0.2")

	site_name = body_name
	site_pos = "0 0 0"
	site_size = "0.02"
	site_rgba = color
	site_type ="sphere"
	site = etree.SubElement(body, 
		'site', 
		name= site_name, 
		pos = site_pos, 
		size=site_size, 
		rgba=site_rgba, 
		type = site_type)

	if make_target:
		target = make_target(body_id, mjcf_model, **{'color':color})
	return body, pos, target

def make_agent(body_id=None, **kwargs):
	body_name = "agent{}".format(body_id)
	joint_name0 = 'agent{}_x'.format(body_id)
	joint_name1 = 'agent{}_y'.format(body_id)
	geom_name = 'agent'

	body = etree.Element('body', name=body_name)
	pos = "0 0 0.03"
	body.set('pos', pos)
	joint = etree.SubElement(body, 'joint', name = joint_name0, type="slide", axis ="1 0 0")
	joint = etree.SubElement(body, 'joint', name=joint_name1, type="slide", axis="0 1 0")

	color = "0 0 0 1"
	geom = etree.SubElement(body, 
		'geom', 
		name=geom_name, 
		type='sphere', 
		size="0.03", 
		rgba=color, 
		contype="1", 
		conaffinity="1", 
		condim="3",
		mass="3")

	return body, pos

def create_floor():
	body_name = "floor0"
	geom_name = 'floor0'

	size = "{} {} 0.01".format(gRange, gRange) 
	floor = etree.Element('body', name=body_name)
	pos = "0 0 0"
	floor.set('pos', pos)

	color = "0 0 0 1"
	geom = etree.SubElement(floor, 
		'geom', 
		name=geom_name, 
		type='plane', 
		size=size, 
		pos = pos,
		material="floor_mat",
		contype="1", 
		conaffinity="1", 
		condim="3",
		)

	return floor


def make_target(body_id, mjcf_model, **kwargs):
	site_name = "target{}".format(body_id)

	site = etree.Element('site', 
		name=site_name)
	pos = get_pos()
	pos = pos + " 0.02"
	site.set('pos', pos)

	color = "1 0 0 1"
	if 'color' in kwargs:
		color = kwargs['color']
	site.set('rgba', color)
	site.set('type', "sphere")
	size = target_size
	if 'size' in kwargs:
		size = kwargs['size']
	site.set('size', size)
	return site


# def make_obstacles():

# 	<body name="floor0" pos="0 0 0">
#             <geom name="floor0" pos="0 0 0" size="0.5 0.5 0.01" material="floor_mat" type="plane" ></geom>
#         </body>


def make_walls():
	wall_range = gRange + 0.05
	sizes = [wall_range, 0.01]
	poses = [0, wall_range]
	walls = []
	for i in range(4):
		body_name = "wall{}".format(i)
		geom_name = 'wall{}'.format(i)

		body = etree.Element('body', name=body_name)
		size = "{} {} 0.1".format(sizes[i%2], sizes[(i+1)%2])
		if i == 2:
			poses = [-1 * poses[j] for j in range(len(poses)) ]

		pos = "{} {} 0.05".format(poses[i%2], poses[(i+1)%2])

		body.set('pos', pos)
		color = "0.2 0.2 0.2 1"
		geom = etree.SubElement(body, 
			'geom', 
			name=geom_name, 
			type='box', 
			size=size, 
			rgba=color, 
			# contype="1", 
			# conaffinity="1", 
			# condim="4",
			mass="5")
		walls.append(body)
	return walls



def main(num_objs, path, model_name, gRange_inp=0.6):
	global gRange
	xml_file = os.path.join(path, "assets", model_name)
	mjcf_model = etree.parse(xml_file)
	worldbody = mjcf_model.find('./worldbody')

	targets = []

	if num_objs:
		for obj in range(num_objs):
			if obj == 0:
				continue
			body, pos, target = make_object(obj, mjcf_model)
			worldbody.append(body)
			targets.append(target)
	else:
		targets.append(make_target(0, mjcf_model))

	for tar in targets:
		worldbody.append(tar)

	contents = etree.tostring(mjcf_model, pretty_print=True)
	contents= minidom.parseString(contents).toprettyxml(indent=" ")

	with open(os.path.join(path, "assets/fetch/", 'output_push.xml'), 'wb') as f:
		f.write(util.to_binary_string(contents))