#pragma once

#include <iostream>
#include <boost/program_options.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>

namespace param
{

inline struct SimulationConfig
{
    std::string robot;
    std::filesystem::path robot_scene;

    int domain_id;
    std::string interface;

    int use_joystick;
    std::string joystick_type;
    std::string joystick_device;
    int joystick_bits;

    int print_scene_information;

    int enable_elastic_band;
    int band_attached_link = 0;

    // --- torque-speed curve ------------------------------------------------------
    // MuJoCo's `motor` actuator applies a flat ctrlrange clamp: the same peak torque is
    // available at any joint speed. Real actuators cannot do that -- back-EMF means
    // available torque falls off as speed rises, which is why a datasheet quotes a
    // no-load speed. Isaac Lab models this (UnitreeActuator::_clip_effort), so a policy
    // trained there learns to work with a joint that brakes itself when spun fast, and
    // then behaves differently here where nothing brakes it.
    //
    // Enabling this makes the bridge apply the same piecewise-linear envelope before
    // handing the torque to MuJoCo:
    //
    //   |dq| <= X1            -> full torque (Y1 if torque and speed agree in sign,
    //                            Y2 if they oppose -- the braking case allows more)
    //   X1 < |dq| < X2        -> falls linearly toward zero
    //   |dq| >= X2            -> zero
    //
    // Off by default, so every existing robot/scene behaves exactly as before.
    struct TorqueSpeedCurve
    {
        std::string pattern;  // matched as a substring of the actuator name; "" = catch-all
        double Y1 = 1e9, Y2 = 1e9, X1 = 1e9, X2 = 1e9;
    };
    int enable_torque_speed_curve = 0;
    std::vector<TorqueSpeedCurve> torque_speed_curves;

    void load_from_yaml(const std::string &filename)
    {
        auto cfg = YAML::LoadFile(filename);
        try
        {
            robot = cfg["robot"].as<std::string>();
            robot_scene = cfg["robot_scene"].as<std::string>();
            domain_id = cfg["domain_id"].as<int>();
            interface = cfg["interface"].as<std::string>();
            use_joystick = cfg["use_joystick"].as<int>();
            joystick_type = cfg["joystick_type"].as<std::string>();
            joystick_device = cfg["joystick_device"].as<std::string>();
            joystick_bits = cfg["joystick_bits"].as<int>();
            print_scene_information = cfg["print_scene_information"].as<int>();
            enable_elastic_band = cfg["enable_elastic_band"].as<int>();

            // Optional: absent key leaves the curve disabled and behaviour unchanged.
            if (cfg["enable_torque_speed_curve"])
            {
                enable_torque_speed_curve = cfg["enable_torque_speed_curve"].as<int>();
            }
            if (cfg["torque_speed_curves"] && cfg["torque_speed_curves"].IsSequence())
            {
                for (const auto &entry : cfg["torque_speed_curves"])
                {
                    TorqueSpeedCurve c;
                    c.pattern = entry["pattern"] ? entry["pattern"].as<std::string>() : "";
                    c.Y1 = entry["Y1"].as<double>();
                    c.Y2 = entry["Y2"] ? entry["Y2"].as<double>() : c.Y1;
                    c.X1 = entry["X1"].as<double>();
                    c.X2 = entry["X2"].as<double>();
                    torque_speed_curves.push_back(c);
                }
            }
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
            exit(EXIT_FAILURE);
        }
    }
} config;

/* ---------- Command Line Parameters ---------- */
namespace po = boost::program_options;

//※ This function must be called at the beginning of main() function
inline po::variables_map helper(int argc, char** argv)
{
    po::options_description desc("Unitree Mujoco");
    desc.add_options()
        ("help,h", "Show help message")
        ("domain_id,i", po::value<int>(&config.domain_id), "DDS domain ID; -i 0")
        ("network,n", po::value<std::string>(&config.interface), "DDS network interface; -n eth0")
        ("robot,r", po::value<std::string>(&config.robot), "Robot type; -r go2")
        ("scene,s", po::value<std::filesystem::path>(&config.robot_scene), "Robot scene file; -s scene_terrain.xml")
    ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    
    if (vm.count("help"))
    {
        std::cout << desc << std::endl;
        exit(0);
    }

    return vm;
}

}