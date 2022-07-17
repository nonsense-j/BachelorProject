import os
import shutil
import re

def decapitalize(str, upper_rest=False):
  return str[:1].lower() + (str[1:].upper() if upper_rest else str[1:])

class FPBuilder:
    """
    A F Prime Auto-Builder.
    """

    def __init__(self, proj_name, fprime_root):

        # model paths --- source
        self.model_path = "lib/FPBuilder/Models"
        # self.model_path = "FPBuilder/Models" # for test

        self.controller_fpp_model_path = "{}/BaseMission/BaseController.fpp".format(self.model_path)
        self.controller_cpp_model_path = "{}/BaseMission/BaseController.cpp".format(self.model_path)
        self.controller_hpp_model_path = "{}/BaseMission/BaseController.hpp".format(self.model_path)
        self.executor_fpp_model_path = "{}/BaseMission/BaseExecutor.fpp".format(self.model_path)
        self.executor_cpp_model_path = "{}/BaseMission/BaseExecutor.cpp".format(self.model_path)
        self.executor_hpp_model_path = "{}/BaseMission/BaseExecutor.hpp".format(self.model_path)
        self.ports_fpp_model_path = "{}/BaseMission/BasePorts.fpp".format(self.model_path)
        self.mission_cmake_model_path = "{}/BaseMission/CMakeLists.txt".format(self.model_path)

        self.sensors_fpp_model_path = "{}/EnvManager/EnvSensors.fpp".format(self.model_path)
        self.env_fpp_model_path = "{}/EnvManager/EnvManager.fpp".format(self.model_path)
        self.env_cpp_model_path = "{}/EnvManager/EnvManager.cpp".format(self.model_path)
        self.env_hpp_model_path = "{}/EnvManager/EnvManager.hpp".format(self.model_path)
        self.env_cmake_model_path = "{}/EnvManager/CMakeLists.txt".format(self.model_path)

        self.sim_fpp_model_path = "{}/SimManager/SimManager.fpp".format(self.model_path)
        self.sim_cpp_model_path = "{}/SimManager/SimManager.cpp".format(self.model_path)
        self.sim_hpp_model_path = "{}/SimManager/SimManager.hpp".format(self.model_path)
        self.sim_cmake_model_path = "{}/SimManager/CMakeLists.txt".format(self.model_path)

        self.start_fpp_model_path = "{}/BaseActuators/BaseStart.fpp".format(self.model_path)
        self.move_fpp_model_path = "{}/BaseActuators/BaseMove.fpp".format(self.model_path)
        self.act_fpp_model_path = "{}/BaseActuators/BaseAct.fpp".format(self.model_path)
        self.actuator_cmake_model_path = "{}/BaseActuators/CMakeLists.txt".format(self.model_path)
        self.actuator_sh_model_path = "{}/BaseActuators/chName.sh".format(self.model_path)

        self.instances_fpp_model_path = "{}/Top/instances.fpp".format(self.model_path)
        self.topology_fpp_model_path = "{}/Top/topology.fpp".format(self.model_path)
        self.ref_cmake_model_path = "{}/CMakeLists.txt".format(self.model_path)

        # fprime locations --- destination
        self.fprime_ref_root = "{}".format(fprime_root)
        self.proj_name = proj_name
        self.proj_name_decap = decapitalize(self.proj_name)
        self.fprime_controller_fpp_path = "{0}/{1}Mission/{1}Controller.fpp".format(self.fprime_ref_root, self.proj_name)
        self.fprime_controller_cpp_path = "{0}/{1}Mission/{1}Controller.cpp".format(self.fprime_ref_root, self.proj_name)
        self.fprime_controller_hpp_path = "{0}/{1}Mission/{1}Controller.hpp".format(self.fprime_ref_root, self.proj_name)
        self.fprime_executor_fpp_path = "{0}/{1}Mission/{1}Executor.fpp".format(self.fprime_ref_root, self.proj_name)
        self.fprime_executor_cpp_path = "{0}/{1}Mission/{1}Executor.cpp".format(self.fprime_ref_root, self.proj_name)
        self.fprime_executor_hpp_path = "{0}/{1}Mission/{1}Executor.hpp".format(self.fprime_ref_root, self.proj_name)
        self.fprime_mission_ports_path = "{0}/{1}Mission/{1}Ports.fpp".format(self.fprime_ref_root, self.proj_name)
        self.fprime_mission_cmake_path = "{0}/{1}Mission/CMakeLists.txt".format(self.fprime_ref_root, self.proj_name)

        self.fprime_sensors_fpp_path = "{}/EnvManager/EnvSensors.fpp".format(self.fprime_ref_root)
        self.fprime_env_fpp_path = "{}/EnvManager/EnvManager.fpp".format(self.fprime_ref_root)
        self.fprime_env_cpp_path = "{}/EnvManager/EnvManager.cpp".format(self.fprime_ref_root)
        self.fprime_env_hpp_path = "{}/EnvManager/EnvManager.hpp".format(self.fprime_ref_root)
        self.fprime_env_cmake_path = "{}/EnvManager/CMakeLists.txt".format(self.fprime_ref_root)

        self.fprime_sim_fpp_path = "{}/SimManager/SimManager.fpp".format(self.fprime_ref_root)
        self.fprime_sim_cpp_path = "{}/SimManager/SimManager.cpp".format(self.fprime_ref_root)
        self.fprime_sim_hpp_path = "{}/SimManager/SimManager.hpp".format(self.fprime_ref_root)
        self.fprime_sim_cmake_path = "{}/SimManager/CMakeLists.txt".format(self.fprime_ref_root)

        self.fprime_start_fpp_path = "{0}/{1}Actuators/{1}Start.fpp".format(self.fprime_ref_root, self.proj_name)
        self.fprime_start_cpp_path = "{0}/{1}Actuators/{1}Start.cpp".format(self.fprime_ref_root, self.proj_name)
        self.fprime_move_fpp_path = "{0}/{1}Actuators/{1}Move.fpp".format(self.fprime_ref_root, self.proj_name)
        self.fprime_move_cpp_path = "{0}/{1}Actuators/{1}Move.cpp".format(self.fprime_ref_root, self.proj_name)
        self.fprime_act_dir_path = "{0}/{1}Actuators".format(self.fprime_ref_root, self.proj_name)
        self.fprime_actuator_cmake_path = "{0}/{1}Actuators/CMakeLists.txt".format(self.fprime_ref_root, self.proj_name)
        self.fprime_actuator_sh_path = "{0}/{1}Actuators/chName.sh".format(self.fprime_ref_root, self.proj_name)

        self.fprime_instances_fpp_path = "{}/Top/instances.fpp".format(self.fprime_ref_root)
        self.fprime_topology_fpp_path = "{}/Top/topology.fpp".format(self.fprime_ref_root)
        self.fprime_ref_cmake_path = "{}/CMakeLists.txt".format(self.fprime_ref_root)

    def buildDirectories(self):
        mission_config_dir = "{0}/{1}Mission/Config".format(self.fprime_ref_root, self.proj_name)
        if not os.path.exists(mission_config_dir):
            os.makedirs(mission_config_dir)

        actuator_dir_path = "{0}/{1}Actuators".format(self.fprime_ref_root, self.proj_name)
        if not os.path.exists(actuator_dir_path):
            os.makedirs(actuator_dir_path)

        env_dir_path = "{}/EnvManager".format(self.fprime_ref_root)
        if not os.path.exists(env_dir_path):
            os.makedirs(env_dir_path)

        sim_dir_path = "{}/SimManager".format(self.fprime_ref_root)
        if not os.path.exists(sim_dir_path):
            os.makedirs(sim_dir_path)

    # ----------------------------------------------------------------------
    # Mission FPP Build
    # ----------------------------------------------------------------------

    def buildPortsFpp(self):
        with open(self.ports_fpp_model_path, 'r') as f:
            ports_fpp_str = f.read()
        ports_fpp_str = ports_fpp_str.replace("#Base#", self.proj_name)
        ports_fpp_str = ports_fpp_str.replace("#base#", self.proj_name_decap)
        with open(self.fprime_mission_ports_path, 'w') as wf:
            wf.write(ports_fpp_str)
        print "Successfully created Mission Ports fpp file in [{}]".format(self.fprime_mission_ports_path)

    def buildControllerFpp(self):
        with open(self.controller_fpp_model_path, 'r') as f:
            controller_fpp_str = f.read()
        controller_fpp_str = controller_fpp_str.replace("#Base#", self.proj_name)
        controller_fpp_str = controller_fpp_str.replace("#base#", self.proj_name_decap)
        with open(self.fprime_controller_fpp_path, 'w') as wf:
            wf.write(controller_fpp_str)
        print "Successfully created Controller fpp file in [{}]".format(self.fprime_controller_fpp_path)

    def buildExecutorFpp(self, sensors, actions, customs):
        with open(self.executor_fpp_model_path, 'r') as f:
            executor_fpp_str = f.read()
        executor_fpp_str = executor_fpp_str.replace("#Base#", self.proj_name)
        executor_fpp_str = executor_fpp_str.replace("#base#", self.proj_name_decap)
        # event: SHOW_TANSITION
        prop_str = ""
        # @Sensor: person[{d}]; @Action: pick_up[{d}] drop[{d}] radio[{d}]; @Customs: carrying_item[{d}];
        add_format_str = "@Sensor:"
        # telemetry $person: I8
        telemetry_str = ""
        # output port patrol_pick_up_out : PatrolAct
        act_outport_str = ""
        # two flags for bettering formatting
        if len(sensors) == 0:
            add_format_str += "None"
        for s in sensors:
            prop_str += "                          ${}: I8 @< sensor value\n".format(s)
            add_format_str += " {}[{{d}}]".format(s)
            telemetry_str += "\n    telemetry ${}: I8\n".format(s)
        add_format_str += "; @Action:"
        if len(actions) == 0:
            add_format_str += "None"
        for a in actions:
            act_outport_str += "\n    output port {0}_{1}_out : {2}Act\n".format(self.proj_name_decap, a, self.proj_name)
            prop_str += "                          ${}: I8 @< action value\n".format(a)
            add_format_str += " {}[{{d}}]".format(a)
            telemetry_str += "\n    telemetry ${}: I8\n".format(a)
        add_format_str += "; @Customs:"
        if len(customs) == 0:
            add_format_str += "None"
        for c in customs:
            prop_str += "                          ${}: I8 @< custom value\n".format(c)
            add_format_str += " {}[{{d}}]".format(c)
            telemetry_str += "\n    telemetry ${}: I8\n".format(c)
        executor_fpp_str = executor_fpp_str.replace("    ## add your output ports for actions ##", act_outport_str)
        executor_fpp_str = executor_fpp_str.replace("                          ## add your props ##\n", prop_str)
        executor_fpp_str = executor_fpp_str.replace("## add your format ##", add_format_str)
        executor_fpp_str = executor_fpp_str.replace("    ## add your telemetry ##", telemetry_str)
        with open(self.fprime_executor_fpp_path, 'w') as wf:
            wf.write(executor_fpp_str)
        print "successfully created Executor fpp file in [{}]".format(self.fprime_executor_fpp_path)

    def buildMissionCMake(self):
        with open(self.mission_cmake_model_path, 'r') as f:
            mission_cmake_str = f.read()
        mission_cmake_str = mission_cmake_str.replace("#Base#", self.proj_name)
        with open(self.fprime_mission_cmake_path, 'w') as wf:
            wf.write(mission_cmake_str)
        print "Successfully created Mission CmakeLists.txt in [{}]".format(self.fprime_mission_cmake_path)

    def buildMissionFpp(self, sensors, actions, customs):
        self.buildPortsFpp()
        self.buildControllerFpp()
        self.buildExecutorFpp(sensors, actions, customs)
        self.buildMissionCMake()

    # ----------------------------------------------------------------------
    # Env Manager Build
    # ----------------------------------------------------------------------

    def buildSensorsFpp(self, sensors):
        with open(self.sensors_fpp_model_path, 'r') as f:
            sensors_fpp_str = f.read()
        sensors_name = ""
        for s in sensors:
            sensors_name += "    ${}\n".format(s)
        sensors_fpp_str = sensors_fpp_str.replace("    ## add your sensors ##\n", sensors_name)
        with open(self.fprime_sensors_fpp_path, 'w') as wf:
            wf.write(sensors_fpp_str)
        print "Successfully created Sensors fpp file in [{}]".format(self.fprime_sensors_fpp_path)

    def buildEnvFpp(self):
        shutil.copyfile(self.env_fpp_model_path, self.fprime_env_fpp_path)
        print "Successfully created EnvManager fpp file in [{}]".format(self.fprime_mission_ports_path)

    def buildEnvCMake(self):
        shutil.copyfile(self.env_cmake_model_path, self.fprime_env_cmake_path)
        print "Successfully created EnvManager CMakeLists.txt in [{}]".format(self.fprime_mission_ports_path)

    def buildEnvManager(self, sensors):
        self.buildSensorsFpp(sensors)
        self.buildEnvFpp()
        self.buildEnvCMake()
        shutil.copyfile(self.env_hpp_model_path, self.fprime_env_hpp_path)
        print "Successfully created EnvManager.hpp in [{}]".format(self.fprime_mission_ports_path)
        with open(self.env_cpp_model_path, 'r') as f:
            env_cpp_str = f.read()
        sensors_handler_code = ""
        for s in sensors:
            sensors_handler_code += "\t\tcase EnvSensor::{}:\n".format(s)
            sensors_handler_code += "\t\t\tval = (value.e == EnvValue::TRUE) ? 1 : 0;\n"
            sensors_handler_code += "\t\t\tthis->envSetout_out(i, \"{}\", val);\n".format(s)
            sensors_handler_code += "\t\t\tbreak;\n"
        env_cpp_str = env_cpp_str.replace("        //## add your sensor handler code##//\n", sensors_handler_code)
        with open(self.fprime_env_cpp_path, 'w') as wf:
            wf.write(env_cpp_str)
        print "Successfully created Sensors fpp file in [{}]".format(self.fprime_sensors_fpp_path)

    # ----------------------------------------------------------------------
    # Simulation Manager Build
    # ----------------------------------------------------------------------

    def buildSimManager(self):
        shutil.copyfile(self.sim_fpp_model_path, self.fprime_sim_fpp_path)
        print "Successfully created SimManager fpp file in [{}]".format(self.fprime_sim_fpp_path)
        shutil.copyfile(self.sim_cpp_model_path, self.fprime_sim_cpp_path)
        print "Successfully created SimManager cpp file in [{}]".format(self.fprime_sim_cpp_path)
        shutil.copyfile(self.sim_hpp_model_path, self.fprime_sim_hpp_path)
        print "Successfully created SimManager hpp file in [{}]".format(self.fprime_sim_cpp_path)
        shutil.copyfile(self.sim_cmake_model_path, self.fprime_sim_cmake_path)
        print "Successfully created SimManager CMakeLists.txt in [{}]".format(self.fprime_sim_cmake_path)

    # ----------------------------------------------------------------------
    # Actuator Build
    # ----------------------------------------------------------------------

    def buildStartFpp(self):
        with open(self.start_fpp_model_path, 'r') as f:
            start_fpp_str = f.read()
        start_fpp_str = start_fpp_str.replace("#Base#", self.proj_name)
        start_fpp_str = start_fpp_str.replace("#base#", self.proj_name_decap)
        with open(self.fprime_start_fpp_path, 'w') as wf:
            wf.write(start_fpp_str)
        if not os.path.exists(self.fprime_start_cpp_path):
            os.mknod(self.fprime_start_cpp_path)
        print "Successfully created Actuator:Start fpp in [{}]".format(self.fprime_start_fpp_path)

    def buildMoveFpp(self):
        with open(self.move_fpp_model_path, 'r') as f:
            move_fpp_str = f.read()
        move_fpp_str = move_fpp_str.replace("#Base#", self.proj_name)
        move_fpp_str = move_fpp_str.replace("#base#", self.proj_name_decap)
        with open(self.fprime_move_fpp_path, 'w') as wf:
            wf.write(move_fpp_str)
        if not os.path.exists(self.fprime_move_cpp_path):
            os.mknod(self.fprime_move_cpp_path)
        print "Successfully created Actuator:Move fpp in [{}]".format(self.fprime_move_fpp_path)

    def buildActuatorCMake(self, actions):
        with open(self.actuator_cmake_model_path, 'r') as f:
            actuator_cmake_str = f.read()
        actuator_cmake_str = actuator_cmake_str.replace("#Base#", self.proj_name)
        actions_cmake_str = ""
        for act in actions:
            actions_cmake_str += "  \"${{CMAKE_CURRENT_LIST_DIR}}/{}_{}.cpp\"\n".format(self.proj_name, act)
            actions_cmake_str += "  \"${{CMAKE_CURRENT_LIST_DIR}}/{}_{}.fpp\"\n".format(self.proj_name, act)
        actuator_cmake_str = actuator_cmake_str.replace("  ## add your actuators ##\n", actions_cmake_str)
        with open(self.fprime_actuator_cmake_path, 'w') as wf:
            wf.write(actuator_cmake_str)
        print "Successfully created Actuator CMakeLists.txt in [{}]".format(self.fprime_actuator_cmake_path)

    def buildActuators(self, actions):
        self.buildStartFpp()
        self.buildMoveFpp()
        # every action refers to a single component
        with open(self.act_fpp_model_path, 'r') as f:
            actuator_fpp_str = f.read()
        actuator_fpp_str = actuator_fpp_str.replace("#Base#", self.proj_name)
        actuator_fpp_str = actuator_fpp_str.replace("#base#", self.proj_name_decap)
        for act in actions:
            fprime_act_fpp_path = "{0}/{1}_{2}.fpp".format(self.fprime_act_dir_path, self.proj_name, act)
            fprime_act_cpp_path = "{0}/{1}_{2}.cpp".format(self.fprime_act_dir_path, self.proj_name, act)
            if not os.path.exists(fprime_act_cpp_path):
                os.mknod(fprime_act_cpp_path)
            with open(fprime_act_fpp_path, 'w') as wf:
                wf.write(actuator_fpp_str.replace("#act#", act))
        print "Successfully created Other Actuators fpp in [{}]".format(self.fprime_actuator_cmake_path)
        self.buildActuatorCMake(actions)
        shutil.copyfile(self.actuator_sh_model_path, self.fprime_actuator_sh_path)

    # ----------------------------------------------------------------------
    # Controller Build
    # ----------------------------------------------------------------------

    def buildController(self):
        # hpp generate
        with open(self.controller_hpp_model_path, 'r') as hf:
            controller_hpp_str = hf.read()
        controller_hpp_str = controller_hpp_str.replace("#Base#", self.proj_name)
        controller_hpp_str = controller_hpp_str.replace("#base#", self.proj_name_decap)
        with open(self.fprime_controller_hpp_path, 'w') as hwf:
            hwf.write(controller_hpp_str)
        # cpp generate
        with open(self.controller_cpp_model_path, 'r') as cf:
            controller_cpp_str = cf.read()
        controller_cpp_str = controller_cpp_str.replace("#Base#", self.proj_name)
        controller_cpp_str = controller_cpp_str.replace("#base#", self.proj_name_decap)
        with open(self.fprime_controller_cpp_path, 'w') as cwf:
            cwf.write(controller_cpp_str)
        print "Successfully created Controller.cpp(hpp) in [{}(hpp)]".format(self.fprime_controller_cpp_path)


    # ----------------------------------------------------------------------
    # Executor Build
    # ----------------------------------------------------------------------

    def buildExecutor(self):
        # hpp generate
        with open(self.executor_hpp_model_path, 'r') as hf:
            executor_hpp_str = hf.read()
        executor_hpp_str = executor_hpp_str.replace("#Base#", self.proj_name)
        executor_hpp_str = executor_hpp_str.replace("#base#", self.proj_name_decap)
        with open(self.fprime_executor_hpp_path, 'w') as hwf:
            hwf.write(executor_hpp_str)
        # cpp generate
        with open(self.executor_cpp_model_path, 'r') as cf:
            executor_cpp_str = cf.read()
        executor_cpp_str = executor_cpp_str.replace("#Base#", self.proj_name)
        executor_cpp_str = executor_cpp_str.replace("#base#", self.proj_name_decap)
        with open(self.fprime_executor_cpp_path, 'w') as cwf:
            cwf.write(executor_cpp_str)
        print "Successfully created Executor.cpp(hpp) in [{}(hpp)]".format(self.fprime_executor_cpp_path)

    # ----------------------------------------------------------------------
    # Mission Config
    # ----------------------------------------------------------------------
    def copyConfig(self, proj_root):
        fprime_mission_config_dir = "{0}/{1}Mission/Config".format(self.fprime_ref_root, self.proj_name)
        # .spec
        spec_file_name = self.proj_name+".spec"
        spec_file_path = os.path.join(proj_root, spec_file_name)
        spec_dest_path = os.path.join(fprime_mission_config_dir, spec_file_name)
        if os.path.isfile(spec_file_path):
            print "-- copying [spec] file from LTLMop to F Prime"
            shutil.copyfile(spec_file_path, spec_dest_path)
        else:
            print "cannot find spec file, try copy manually to {}".format(fprime_mission_config_dir)

        # .aut
        aut_file_name = self.proj_name + ".aut"
        aut_file_path = os.path.join(proj_root, aut_file_name)
        aut_dest_path = os.path.join(fprime_mission_config_dir, aut_file_name)
        if os.path.isfile(aut_file_path):
            print "-- copying [aut] file from LTLMop to F Prime"
            shutil.copyfile(aut_file_path, aut_dest_path)
        else:
            print "cannot find aut file, try copy manually to {}".format(fprime_mission_config_dir)

        # decomposed region
        region_file_name = self.proj_name + "_decomposed.regions"
        region_file_path = os.path.join(proj_root, region_file_name)
        region_dest_path = os.path.join(fprime_mission_config_dir, region_file_name)
        if os.path.isfile(region_file_path):
            print "-- copying [decomposed region] file from LTLMop to F Prime"
            shutil.copyfile(region_file_path, region_dest_path)
        else:
            print "cannot find region file, try copy manually to {}".format(fprime_mission_config_dir)

    # ----------------------------------------------------------------------
    # Top Build
    # ----------------------------------------------------------------------
    def buildRefCMake(self, add_write=False):
        source_path = self.ref_cmake_model_path
        if add_write:
            source_path = self.fprime_ref_cmake_path
        with open(source_path, 'r') as f:
            ref_cmake_str = f.read()
        add_cmake_str = "add_fprime_subdirectory(\"${CMAKE_CURRENT_LIST_DIR}/" + self.proj_name + "Mission/\")\n"
        add_cmake_str += "add_fprime_subdirectory(\"${CMAKE_CURRENT_LIST_DIR}/" + self.proj_name + "Actuators/\")\n"
        add_cmake_str += "# add more components here\n"
        ref_cmake_str = ref_cmake_str.replace("# add more components here\n", add_cmake_str)
        with open(self.fprime_ref_cmake_path, 'w') as wf:
            wf.write(ref_cmake_str)
        print "Successfully supplemented Ref/CmakeLists.txt"

    def buildInstances(self, actions, add_write = False):
        source_path = self.instances_fpp_model_path
        if add_write:
            source_path = self.fprime_instances_fpp_path
        with open(source_path, 'r') as f:
            instances_str = f.read()
        pattern = re.compile(r'base id (0x.+) \\\n.*\n.*\n.*\n  # add more active instances here')
        active_index = int(re.search(pattern, instances_str).group(1), 16)
        single_instance_str = "\n  instance {0}: Ref.{1} base id {2} \\\n"
        single_instance_str += "    queue size Default.queueSize \\\n"
        single_instance_str += "    stack size Default.stackSize \\\n"
        single_instance_str += "    priority 100\n"
        sup_instances_str = ""
        # add baseController
        active_index += 0x100
        ins_name = self.proj_name_decap+"Controller"
        comp_name = self.proj_name+"Controller"
        index_hex = "%X" % active_index
        index_hex = "0x" + index_hex.rjust(4, "0")
        sup_instances_str += single_instance_str.format(ins_name, comp_name, index_hex)
        # add baseExecutor
        active_index += 0x100
        ins_name = self.proj_name_decap + "Executor"
        comp_name = self.proj_name + "Executor"
        index_hex = "%X" % active_index
        index_hex = "0x" + index_hex.rjust(4, "0")
        sup_instances_str += single_instance_str.format(ins_name, comp_name, index_hex)
        sup_instances_str += "  # add more active instances here\n"
        instances_str = instances_str.replace("  # add more active instances here\n", sup_instances_str)

        # add baseStart
        pattern = re.compile(r'base id (0x.+)\n  # add more passive instances here')
        passive_index = int(re.search(pattern, instances_str).group(1), 16)+0x100
        index_hex = "%X" % passive_index
        index_hex = "0x" + index_hex.rjust(4, "0")
        ins_name = self.proj_name_decap + "Start"
        comp_name = self.proj_name + "Start"
        passive_add_str = "\n  instance {0}: Ref.{1} base id {2}\n".format(ins_name, comp_name, index_hex)
        # add baseMove
        passive_index += 0x100
        ins_name = self.proj_name_decap + "Move"
        comp_name = self.proj_name + "Move"
        index_hex = "%X" % passive_index
        index_hex = "0x" + index_hex.rjust(4, "0")
        passive_add_str += "\n  instance {0}: Ref.{1} base id {2}\n".format(ins_name, comp_name, index_hex)
        for act in actions:
            passive_index += 0x100
            ins_name = "{}_{}".format(self.proj_name_decap, act)
            comp_name = "{}_{}".format(self.proj_name, act)
            index_hex = "%X" % passive_index
            index_hex = "0x" + index_hex.rjust(4, "0")
            passive_add_str += "\n  instance {0}: Ref.{1} base id {2}\n".format(ins_name, comp_name, index_hex)

        passive_add_str += "  # add more passive instances here\n"
        instances_str = instances_str.replace("  # add more passive instances here\n", passive_add_str)

        with open(self.fprime_instances_fpp_path, 'w') as wf:
            wf.write(instances_str)
        print "Successfully supplemented Ref/Top/instances.fpp"

    def buildTopology(self, actions, add_write=False):
        source_path = self.topology_fpp_model_path
        if add_write:
            source_path = self.fprime_topology_fpp_path
        with open(source_path, 'r') as f:
            topology_str = f.read()
        topology_str = topology_str.replace("#Base#", self.proj_name)
        topology_str = topology_str.replace("#base#", self.proj_name_decap)
        sup_topology_str = ""
        # instance baseController
        ins_pattern = "    instance {}\n"
        ins_name = self.proj_name_decap + "Controller"
        sup_topology_str += ins_pattern.format(ins_name)
        ins_name = self.proj_name_decap + "Executor"
        sup_topology_str += ins_pattern.format(ins_name)
        ins_name = self.proj_name_decap + "Start"
        sup_topology_str += ins_pattern.format(ins_name)
        ins_name = self.proj_name_decap + "Move"
        sup_topology_str += ins_pattern.format(ins_name)
        for act in actions:
            ins_name = "{}_{}".format(self.proj_name_decap, act)
            sup_topology_str += ins_pattern.format(ins_name)
        sup_topology_str += "    # add more instances here\n"
        topology_str = topology_str.replace("    # add more instances here\n", sup_topology_str)
        # connections supplement
        connection_str = "    connections {0} {{\n\n"
        connection_str += "      envManager.envSetout -> {1}Executor.envSetin\n"
        connection_str += "      simManager.simControlout -> {1}Controller.simControlin\n\n"
        connection_str += "      {1}Controller.{1}Wakeout -> {1}Executor.{1}Wakein\n"
        connection_str += "      {1}Controller.{1}ExeCtrlout -> {1}Executor.{1}ExeCtrlin\n"
        connection_str += "      {1}Executor.{1}Callbackout -> {1}Controller.{1}Callbackin\n\n"
        connection_str += "      {1}Executor.{1}Startout  -> {1}Start.{1}Startin\n"
        connection_str += "      {1}Executor.{1}Moveout  -> {1}Move.{1}Movein\n"
        # connection_str += "      {1}Move.{1}ActStatusout  -> {1}Executor.{1}ActStatusin\n"
        connection_str = connection_str.format(self.proj_name, self.proj_name_decap)
        for act in actions:
            # patrol_pick_up_in
            connection_str += "      {0}Executor.{0}_{1}_out  -> {0}_{1}.{0}_{1}_in\n".format(self.proj_name_decap, act)
            # connection_str += "      {0}_{1}.{0}ActStatusout  -> {0}Executor.{0}ActStatusin\n".format(self.proj_name_decap, act)
        connection_str += "\n    }\n\n    # add more connections here\n"
        topology_str = topology_str.replace("    # add more connections here\n", connection_str)

        with open(self.fprime_topology_fpp_path, 'w') as wf:
            wf.write(topology_str)
        print "Successfully supplemented Ref/Top/topology.fpp"

    def buildTop(self, actions, add_write=False):
        self.buildRefCMake(add_write)
        self.buildInstances(actions, add_write)
        self.buildTopology(actions, add_write)

    # ----------------------------------------------------------------------
    # Clean All -- not influencing the EnvManager and SimManager
    # ----------------------------------------------------------------------
    def cleanAll(self):
        if self.proj_name is None:
            print "Please import or create a specification project first..."
            return
        for name in os.listdir(self.fprime_ref_root):
            path = os.path.join(self.fprime_ref_root, name)
            if os.path.isdir(path) and name.endswith(("Actuators", "Mission", "EnvManager", "SimManager")):
                print "Cleaning: File ["+path+"] is removed"
                shutil.rmtree(path)
        shutil.copyfile(self.instances_fpp_model_path, self.fprime_instances_fpp_path)
        print "Cleaning: File [Ref/Top/instances.fpp] is reset"
        shutil.copyfile(self.topology_fpp_model_path, self.fprime_topology_fpp_path)
        print "Cleaning: File [Ref/Top/topology.fpp] is reset"
        shutil.copyfile(self.ref_cmake_model_path, self.fprime_ref_cmake_path)
        print "Cleaning: File [Ref/CMakeLists.txt] is reset"

    # ----------------------------------------------------------------------
    # Top Build
    # ----------------------------------------------------------------------
    def buildAll(self, sensors, actions, customs, add_write=False, init_clean=True):
        if self.proj_name is None:
            print "Please import or create a specification project first..."
            return
        if init_clean:
            self.cleanAll()
        self.buildDirectories()
        self.buildMissionFpp(sensors, actions, customs)
        self.buildEnvManager(sensors)
        self.buildSimManager()
        self.buildActuators(actions)
        self.buildController()
        self.buildExecutor()
        self.buildTop(actions, add_write)
