# Tether Plugin

An open source gazebo plugin that allows the simulation of a tether (non rigid-body) inside gazebo. This tether can be connected to 2 vehicles (gazebo models) at the same time. 

If you are using this code for research and development for your publication, please cite:
```
@mastersthesis{AAntunes,
  title={Cooperative Path-Following Control of Aerial and Marine Vehicles},
  author={Antunes, Antonio},
  school={Instituto Superior Tecnico},
  url="https://fenix.tecnico.ulisboa.pt/cursos/meec21/dissertacao/1128253548923180",
  year={2022},
  month={December},
  type={M.Sc. thesis}
}
```

## How to use:
First launch the 2 vehicle models inside gazebo. For instance, if you are using the DSOR's/ISR gazebo simulation code available at [https://github.com/dsor-isr/dsor_simulation](https://github.com/dsor-isr/dsor_simulation), you just need to:

1. Launch the simulation world in gazebo
```
roslaunch simulation_bringup start_scenario.launch
```

2. Launch vehicle 1
```
roslaunch simulation_bringup start_vehicle.launch name:=mvector
```

3. Launch vehicle 2
```
roslaunch simulation_bringup start_vehicle.launch name:=myellow
```

4. Launch the tether that will attach to that vehicle
```
roslaunch tether_plugin tether.launch vehicle_1:=mvector0 vehicle_2:=myellow0
```

You should then see something like this:

<p align = "center">
  <img src="img/Example 1.png" width = "426" height = "240" border="5" />
</p>


It also works with PX4 models. Just select vehicle 1 to be your drone! Here is a showcase example:

<p align = "center">
  <img src="img/demo_cut.gif" width = "426" height = "240" border="5" />
</p>


## Developed by:
António Antunes <antonioantunes5@tecnico.ulisboa.pt>

Dynamical Systems and Ocean Robotics Lab (DSOR Lab)

Institute for Systems and Robotics (ISR-Lisbon)

Instituto Superior Técnico (IST)

## License:
TetherPlugin is open-sourced under the MIT [license](LICENSE). See the LICENSE file for details. The scripts directory was adapted from [PX4-SITL_gazebo](https://github.com/PX4/PX4-SITL_gazebo), and if follows their own license agreement.