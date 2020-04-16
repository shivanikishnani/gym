import os
def write_to_xml2(num, model_path, initial_path):
    model_path = os.path.join(os.path.dirname(__file__)[:-5], 'assets', model_path)
    initial_path = os.path.join(os.path.dirname(__file__)[:-5], 'assets', initial_path)

    myfile = open(model_path, "w")
    initial_data = open(initial_path, "r")

    colors = { 0 : '1 0 0 1', 
        1 : '1 1 0 1', 
        2 : '1 0.5 0.5 1', 
        3 : '0.5 1 1 1',  
        4 : '0.8 0.8 1 1' 
        }

    target = '\n\t\t\t<site name="target{}" pos="2 2 0.03" size="0.2 0.2 0.2" rgba="{}" type="sphere"></site>'
    body = '\t\t<body name="object{}" pos="0.2 0.4 0.03"> \n\
              <geom size="0.4 0.4 0.4" type="box" condim="3" rgba="{}"   name="object{}" material="block_mat"></geom> \n\
              <inertial pos="0 0 0" mass="0.01" diaginertia="0.0001 0.0001 0.0001"/>\n\
              <joint name="object{}:joint" type="free" damping="0.001"></joint> \n\
              <site name="object{}" pos="0 0 0" size="0.075 0.075 0.075" rgba="{}" type="sphere"></site> \n\
            </body> \n'
    targets = ""
    bodies = ""

    for i in range(num):
        targets += target.format(i, colors[i])
        bodies += body.format(i, colors[i], i, i, i, colors[i])
    if num == 0:
        targets = '\n<site name="target0" pos="2 2 0.03" size="0.06 0.06 0.06" rgba="1 0 0 1" type="sphere"></site>'
    targets += '\t\t</body>\n'
    bodies += '\
        <light directional="true" ambient="0.2 0.2 0.2" diffuse="0.8 0.8 0.8" specular="0.3 0.3 0.3" \
        castshadow="false" pos="0 0 4" dir="0 0 -1" name="light0"></light> \n\
        </worldbody>\n\
        <actuator>\n\
            <!-- Those are just dummy actuators for providing ranges -->\n\
            <motor joint="agentx" ctrlrange="-1 1" ctrllimited="true" />\n\
            <motor joint="agenty" ctrlrange="-1 1" ctrllimited="true" />\n\
            <motor joint="agentRot" ctrlrange="-0.25 0.25" ctrllimited="true" />\n\
        </actuator>\n\
    </mujoco>'

    [myfile.write(line) for line in initial_data]
    myfile.write(targets)
    myfile.write(bodies)

'''
<joint name="object0{}" type="slide" axis="1 0 0" pos="0 0 0" />\n\
<joint name="object1{}" type="slide" axis="0 1 0" pos="0 0 0" />\n\
'''

def write_to_xml(num, model_path, initial_path):
    model_path = os.path.join(os.path.dirname(__file__)[:-5], 'assets', model_path)
    initial_path = os.path.join(os.path.dirname(__file__)[:-5], 'assets', initial_path)

    myfile = open(model_path, "w")
    initial_data = open(initial_path, "r")

    colors = { 0 : '1 0 0 1', 
        1 : '1 1 0 1', 
        2 : '1 0.5 0.5 1', 
        3 : '0.5 1 1 1',  
        4 : '0.8 0.8 1 1' 
        }

    target = '\n\t\t\t<site name="target{}" pos="2 2 0.03" size="0.2 0.2 0.2" rgba="{}" type="sphere"></site>'
    body = '\t\t<body name="object{}" pos="0.2 0.4 0.03"> \n\
              <geom size="0.4 0.4 0.4" type="box" condim="3" rgba="{}"   name="object{}" material="block_mat"></geom> \n\
              <inertial pos="0 0 0" mass="0.01" diaginertia="0.0001 0.0001 0.0001"/>\n\
              <joint name="object{}:joint" type="free" damping="0.001"></joint> \n\
              <site name="object{}" pos="0 0 0" size="0.075 0.075 0.075" rgba="{}" type="sphere"></site> \n\
            </body> \n'
    targets = ""
    bodies = ""

    for i in range(num):
        targets += target.format(i, colors[i])
        bodies += body.format(i, colors[i], i, i, i, colors[i])
    if num == 0:
        targets = '\n<site name="target0" pos="2 2 0.03" size="0.06 0.06 0.06" rgba="1 0 0 1" type="sphere"></site>'
    targets += '\t\t</body>\n'
    bodies += '\
        <light directional="true" ambient="0.2 0.2 0.2" diffuse="0.8 0.8 0.8" specular="0.3 0.3 0.3" \
        castshadow="false" pos="0 0 4" dir="0 0 -1" name="light0"></light> \n\
        </worldbody>\n\
    </mujoco>'

    [myfile.write(line) for line in initial_data]
    myfile.write(targets)
    myfile.write(bodies)

#     <joint name='agentRot' type='hinge' axis='0 0 1' pos='0 0 0' limited="false" />
# def write_to_xml(num, model_path, initial_path):
#     model_path = os.path.join(os.path.dirname(__file__)[:-5], 'assets', model_path)
#     initial_path = os.path.join(os.path.dirname(__file__)[:-5], 'assets', initial_path)

#     myfile = open(model_path, "w")
#     initial_data = open(initial_path, "r")

#     colors = { 0 : '1 0 0 1', 
#         1 : '1 1 0 1', 
#         2 : '1 0.5 0.5 1', 
#         3 : '0.5 1 1 1',  
#         4 : '0.8 0.8 1 1' 
#         }

#     target = '\n\t\t\t<site name="target{}" pos="2 2 0.03" size="0.2 0.2 0.2" rgba="{}" type="sphere"></site>'
#     body = '\t\t<body name="object{}" pos="0.025 0.025 0.025"> \n\
#             <joint name="object0:joint" type="free" damping="0.01"></joint>
#             <geom size="0.025 0.025 0.025" type="box" condim="3" name="object0" material="block_mat" mass="2"></geom>
#             <site name="object0" pos="0 0 0" size="0.02 0.02 0.02" rgba="1 0 0 1" type="sphere"></site>
#             </body> \n'
#     targets = ""
#     bodies = ""

#     for i in range(num):
#         targets += target.format(i, colors[i])
#         bodies += body.format(i, colors[i], i, i, i, colors[i])
#     if num == 0:
#         targets = '\n<site name="target0" pos="2 2 0.03" size="0.06 0.06 0.06" rgba="1 0 0 1" type="sphere"></site>'
#     targets += '\t\t</body>\n'
#     bodies += '\
#         <light directional="true" ambient="0.2 0.2 0.2" diffuse="0.8 0.8 0.8" specular="0.3 0.3 0.3" \
#         castshadow="false" pos="0 0 4" dir="0 0 -1" name="light0"></light> \n\
#         </worldbody>\n\
#     </mujoco>'

#     [myfile.write(line) for line in initial_data]
#     myfile.write(targets)
#     myfile.write(bodies)


    
#         <body name="object0" pos="0.025 0.025 0.025">
#             <joint name="object0:joint" type="free" damping="0.01"></joint>
#             <geom size="0.025 0.025 0.025" type="box" condim="3" name="object0" material="block_mat" mass="2"></geom>
#             <site name="object0" pos="0 0 0" size="0.02 0.02 0.02" rgba="1 0 0 1" type="sphere"></site>
#         </body>


# 