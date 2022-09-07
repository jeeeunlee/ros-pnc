#include <../my_utils/Configuration.h>
#include <my_simulator/Dart/Magneto/MagnetoWorldNode.hpp>
#include <my_pnc/MagnetoPnC/MagnetoDefinition.hpp>
#include <my_utils/IO/IOUtilities.hpp>
#include <dart/dart.hpp>
#include <dart/gui/osg/osg.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <dart/utils/utils.hpp>
#include <random>

double DEG2RAD = 0.017453293;

void displayJointFrames(const dart::simulation::WorldPtr& world,
                        const dart::dynamics::SkeletonPtr& robot) {
    for (std::size_t i = 0; i < robot->getNumBodyNodes(); ++i) {
        dart::dynamics::BodyNode* bn = robot->getBodyNode(i);
        for (std::size_t j = 0; j < bn->getNumChildJoints(); ++j) {
            const dart::dynamics::Joint* joint = bn->getChildJoint(j);
            const Eigen::Isometry3d offset =
                joint->getTransformFromParentBodyNode();

            dart::gui::osg::InteractiveFramePtr frame =
                std::make_shared<dart::gui::osg::InteractiveFrame>(
                    bn, joint->getName() + "/frame", offset);

            for (const auto type : {dart::gui::osg::InteractiveTool::ANGULAR,
                                    dart::gui::osg::InteractiveTool::PLANAR})
                for (std::size_t i = 0; i < 3; ++i)
                    frame->getTool(type, i)->setEnabled(false);

            world->addSimpleFrame(frame);
        }
    }
}


void displayLinkFrames(const dart::simulation::WorldPtr& world,
                        const dart::dynamics::SkeletonPtr& robot) {

    // -- DISPLAY CERTAIN LINKS
    std::vector<std::string> LinkNametoDisplay;
    LinkNametoDisplay.clear();
    LinkNametoDisplay.push_back("AL_foot_link");
    // LinkNametoDisplay.push_back("AR_foot_link");
    // LinkNametoDisplay.push_back("BL_foot_link");
    // LinkNametoDisplay.push_back("BR_foot_link");
    LinkNametoDisplay.push_back("base_link");

    for(int i=0; i<LinkNametoDisplay.size(); i++) {
        dart::dynamics::BodyNode* bn = robot->getBodyNode(LinkNametoDisplay[i]);
        dart::gui::osg::InteractiveFramePtr frame =
            std::make_shared<dart::gui::osg::InteractiveFrame>(
                bn, bn->getName() + "/frame");

        for (size_t i = 0; i < 3; ++i)
            for (size_t j = 0; j < 3; ++j)
                frame->getTool((dart::gui::osg::InteractiveTool::Type)(i), j)
                    ->setEnabled(false);

        world->addSimpleFrame(frame);
    }

    // -- DISPLAY GROUND LINK
    // dart::dynamics::SkeletonPtr ground = world->getSkeleton("ground_skeleton");
    // dart::dynamics::BodyNode* bn = ground->getBodyNode("world_frame");
    // dart::gui::osg::InteractiveFramePtr frame =
    // std::make_shared<dart::gui::osg::InteractiveFrame>(bn, bn->getName() + "/frame");

    // for (const auto type : {dart::gui::osg::InteractiveTool::ANGULAR,
    //                         dart::gui::osg::InteractiveTool::PLANAR})
    //     for (std::size_t i = 0; i < 3; ++i)
    //         frame->getTool(type, i)->setEnabled(false);

    // world->addSimpleFrame(frame);
}        

class OneStepProgress : public osgGA::GUIEventHandler {
   public:
    OneStepProgress(MagnetoWorldNode* worldnode) : worldnode_(worldnode) {}

    /** Deprecated, Handle events, return true if handled, false otherwise. */
    virtual bool handle(const osgGA::GUIEventAdapter& ea,
                        osgGA::GUIActionAdapter& /*aa*/) {

        if (ea.getEventType() == osgGA::GUIEventAdapter::KEYUP) {            
            if ( ea.getKey() == 'f') {
                int numStepProgress(50);                
                for (int i = 0; i < numStepProgress; ++i) {
                    worldnode_->customPreStep();
                    worldnode_->getWorld()->step();
                    worldnode_->customPostStep();
                }
                return true;
            } else {
                uint16_t button_pressed = ea.getKey();
                std::cout << "button(" << (char)button_pressed << ")  pressed handled @ Main.cpp" << std::endl;
                worldnode_->enableButtonFlag(button_pressed);
            }                     
        }
        return false;
    }
    MagnetoWorldNode* worldnode_;
};

void _setJointLimitConstraint(dart::dynamics::SkeletonPtr robot) {
    for (int i = 0; i < robot->getNumJoints(); ++i) {
        dart::dynamics::Joint* joint = robot->getJoint(i);
        joint->setPositionLimitEnforced(true);
    }
}

void _setTransparency(dart::dynamics::SkeletonPtr robot) {
    for (int i = 0; i < robot->getNumBodyNodes(); ++i) {
        dart::dynamics::BodyNodePtr bn = robot->getBodyNode(i);
        auto sns = bn->getShapeNodesWith<dart::dynamics::VisualAspect>();
        for (auto sn : sns) {
            sn->getShape()->addDataVariance(
                dart::dynamics::Shape::DYNAMIC_COLOR);
            sn->getVisualAspect()->setAlpha(0.4);
        }
    }
}


Eigen::Vector3d _getOrientation(const double& qz, const double& qy, const double& qx, const double& rot_z){
    Eigen::Matrix3d m;

    m = Eigen::AngleAxisd(qz, Eigen::Vector3d::UnitZ())
        * Eigen::AngleAxisd(qy, Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(qx, Eigen::Vector3d::UnitX())
        * Eigen::AngleAxisd(rot_z, Eigen::Vector3d::UnitZ());

    return m.eulerAngles(2, 1, 0); 
}

void _setInitialConfiguration(dart::dynamics::SkeletonPtr robot, 
                            const Eigen::VectorXd& q_v = Eigen::VectorXd::Zero(6), 
                            const double& q_rz = 0.0) {

    Eigen::VectorXd q = robot->getPositions();
    Eigen::VectorXd base_link_init = q_v;
    Eigen::Vector3d eulerzyx = _getOrientation(base_link_init[3], 
                                base_link_init[4], 
                                base_link_init[5], 
                                q_rz*DEG2RAD );
   
    base_link_init.tail(3) = eulerzyx;

    // set initial configuration
    double coxa_joint_init = 0.0; //2./10.*M_PI_2; // 0.0 
    double femur_joint_init =  -1./10.*M_PI_2; //initFemurRand(gen); // -1./10.*M_PI_2;
    double tibia_joint_init = - M_PI_2 - femur_joint_init;

    base_link_init[2] = base_link_init[2] - 0.25*femur_joint_init; // 0.085 // : 1./10.*M_PI_2;

    q.setZero();
    q.segment(0,6) = base_link_init.head(6);   

    q[MagnetoDoF::AL_coxa_joint] = coxa_joint_init; 
    q[MagnetoDoF::AL_femur_joint] = femur_joint_init;
    q[MagnetoDoF::AL_tibia_joint] = tibia_joint_init;

    q[MagnetoDoF::AR_coxa_joint] = coxa_joint_init;  
    q[MagnetoDoF::AR_femur_joint] = femur_joint_init;
    q[MagnetoDoF::AR_tibia_joint] = tibia_joint_init;

    q[MagnetoDoF::BL_coxa_joint] = coxa_joint_init;  
    q[MagnetoDoF::BL_femur_joint] = femur_joint_init;
    q[MagnetoDoF::BL_tibia_joint] = tibia_joint_init;

    q[MagnetoDoF::BR_coxa_joint] = coxa_joint_init;  
    q[MagnetoDoF::BR_femur_joint] = femur_joint_init;
    q[MagnetoDoF::BR_tibia_joint] = tibia_joint_init;

    q[MagnetoDoF::CL_coxa_joint] = coxa_joint_init;  
    q[MagnetoDoF::CL_femur_joint] = femur_joint_init;
    q[MagnetoDoF::CL_tibia_joint] = tibia_joint_init;

    q[MagnetoDoF::CR_coxa_joint] = coxa_joint_init;  
    q[MagnetoDoF::CR_femur_joint] = femur_joint_init;
    q[MagnetoDoF::CR_tibia_joint] = tibia_joint_init;

    q[MagnetoDoF::AL_coxa_joint] = 1./10.*M_PI_2;
    q[MagnetoDoF::AR_coxa_joint] = 2./10.*M_PI_2;
    q[MagnetoDoF::BL_coxa_joint] = -2./10.*M_PI_2;
    q[MagnetoDoF::BR_coxa_joint] = -3./10.*M_PI_2;
    q[MagnetoDoF::CL_coxa_joint] = -2./10.*M_PI_2;
    q[MagnetoDoF::CR_coxa_joint] = 2./10.*M_PI_2;

    robot->setPositions(q);
}

int main(int argc, char** argv) {
    double servo_rate;
    bool isRecord;
    bool b_show_joint_frame;
    bool b_show_link_frame;
    std::string ground_file;
    Eigen::VectorXd q_floating_base_init = Eigen::VectorXd::Zero(6);
    double q_temp;
    double coef_fric;

    Eigen::VectorXd view_eye = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd view_center = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd view_up = Eigen::VectorXd::Zero(3);
    double q_rz;
    //std::ostringstream ground_file;
    try {
        YAML::Node simulation_cfg =
            YAML::LoadFile(THIS_COM "config/Magneto/SIMULATION_DATAGATHERING.yaml");
        my_utils::readParameter(simulation_cfg, "servo_rate", servo_rate);
        my_utils::readParameter(simulation_cfg, "is_record", isRecord);
        my_utils::readParameter(simulation_cfg, "show_joint_frame", b_show_joint_frame);
        my_utils::readParameter(simulation_cfg, "show_link_frame", b_show_link_frame);
        my_utils::readParameter(simulation_cfg, "ground", ground_file);

        my_utils::readParameter(simulation_cfg, "initial_pose", q_floating_base_init); 
        my_utils::readParameter(simulation_cfg, "initial_rz", q_rz); 
        
        my_utils::readParameter(simulation_cfg, "friction", coef_fric);

        my_utils::readParameter(simulation_cfg, "view_eye", view_eye);
        my_utils::readParameter(simulation_cfg, "view_center", view_center);
        my_utils::readParameter(simulation_cfg, "view_up", view_up);              

    } catch (std::runtime_error& e) {
        std::cout << "Error reading parameter [" << e.what() << "] at file: ["
                  << __FILE__ << "]" << std::endl
                  << std::endl;
    }

    my_utils::pretty_print(q_floating_base_init, std::cout, "q_floating_base_init");    

    // =========================================================================
    // Generate world and add skeletons
    // =========================================================================
    dart::simulation::WorldPtr world(new dart::simulation::World);
    dart::utils::DartLoader urdfLoader;
    ground_file.insert(0, THIS_COM);
    std::cout << ground_file << std::endl;
    dart::dynamics::SkeletonPtr ground = urdfLoader.parseSkeleton(
        ground_file);
    dart::dynamics::SkeletonPtr robot = urdfLoader.parseSkeleton(
        // THIS_COM "robot_description/Robot/Magneto/MagnetoSim_Dart.urdf");
        THIS_COM "robot_description/Robot/Magneto/magneto_hexa.urdf");        
    world->addSkeleton(ground);
    world->addSkeleton(robot);

    // =========================================================================
    // Friction & Restitution Coefficient
    // =========================================================================
    double friction(coef_fric); // maximum tangential force not mu
    double restit(0.0);
    ground->getBodyNode("ground_link")->setFrictionCoeff(friction);
    robot->getBodyNode("BL_foot_link")->setFrictionCoeff(friction);
    robot->getBodyNode("AL_foot_link")->setFrictionCoeff(friction);
    robot->getBodyNode("AR_foot_link")->setFrictionCoeff(friction);
    robot->getBodyNode("BR_foot_link")->setFrictionCoeff(friction);

    robot->getBodyNode("BL_foot_link_3")->setFrictionCoeff(friction);
    robot->getBodyNode("AL_foot_link_3")->setFrictionCoeff(friction);
    robot->getBodyNode("AR_foot_link_3")->setFrictionCoeff(friction);
    robot->getBodyNode("BR_foot_link_3")->setFrictionCoeff(friction);

    Eigen::Vector3d gravity(0.0, 0.0, -9.81);
    world->setGravity(gravity);
    world->setTimeStep(servo_rate);

    // =========================================================================
    // Display Joints Frame
    // =========================================================================
    if (b_show_joint_frame) displayJointFrames(world, robot);
    if (b_show_link_frame) displayLinkFrames(world, robot);

    // =========================================================================
    // Initial configuration
    // =========================================================================
    
    _setInitialConfiguration(robot, q_floating_base_init, q_rz);
    // TODO
    // =========================================================================
    // Enabel Joit Limits
    // =========================================================================
    _setJointLimitConstraint(robot);

    // =========================================================================
    // Set Transparency
    // =========================================================================
    // _setTransparencye(robot);

    // =========================================================================
    // Wrap a worldnode
    // =========================================================================
    osg::ref_ptr<MagnetoWorldNode> node;
    node = new MagnetoWorldNode(world);
    node->setNumStepsPerCycle(30);


    // =========================================================================
    // Create and Set Viewer
    // =========================================================================
    dart::gui::osg::Viewer viewer;
    viewer.addWorldNode(node);
    viewer.simulate(false);
    viewer.switchHeadlights(false);
    ::osg::Vec3 p1(1.0, 0.2, 1.0);
    p1 = p1 * 0.7;
    viewer.getLightSource(0)->getLight()->setPosition(
        ::osg::Vec4(p1[0], p1[1], p1[2], 0.0));
    viewer.getCamera()->setClearColor(osg::Vec4(0.93f, 0.95f, 1.0f, 0.95f));
    viewer.getCamera()->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    viewer.addEventHandler(new OneStepProgress(node));

    if (isRecord) {
        std::cout << "[Video Record Enable]" << std::endl;
        viewer.record(THIS_COM "/ExperimentVideo");
    }

    // viewer.setUpViewInWindow(0, 0, 2880, 1800);
    viewer.setUpViewInWindow(1440, 0, 500, 500);
    // viewer.getCameraManipulator()->setHomePosition(
    //     ::osg::Vec3(5.14, 2.28, 3.0), ::osg::Vec3(0.0, 0.2, 0.5),
    //     ::osg::Vec3(0.0, 0.0, 1.0)); // eye, center, up

    // std::cout<< "view" << std::endl;
    // std::cout<< view_eye.transpose() << std::endl;
    // std::cout<< view_center.transpose() << std::endl;
    // std::cout<< view_up.transpose() << std::endl;

    viewer.getCameraManipulator()->setHomePosition(
            ::osg::Vec3(view_eye[0], view_eye[1], view_eye[2]),
            ::osg::Vec3(view_center[0], view_center[1], view_center[2]),
            ::osg::Vec3(view_up[0], view_up[1], view_up[2]) );
    viewer.setCameraManipulator(viewer.getCameraManipulator());
    viewer.run();
}
