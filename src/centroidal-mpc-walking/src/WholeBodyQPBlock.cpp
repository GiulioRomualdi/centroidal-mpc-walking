/**
 * @file WholeBodyQPBlock.cpp
 * @authors Giulio Romualdi
 * @copyright This software may be modified and distributed under the terms of the GNU Lesser
 * General Public License v2.1 or any later version.
 */

#include <yarp/os/RFModule.h>

#include <iDynTree/Core/SpatialMomentum.h>
#include <iDynTree/Model/Model.h>

#include <BipedalLocomotion/Conversions/ManifConversions.h>
#include <BipedalLocomotion/FloatingBaseEstimators/ModelComputationsHelper.h>
#include <BipedalLocomotion/Math/Constants.h>
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>
#include <BipedalLocomotion/RobotInterface/YarpHelper.h>
#include <BipedalLocomotion/System/Clock.h>
#include <BipedalLocomotion/TextLogging/Logger.h>
#include <CentroidalMPCWalking/WholeBodyQPBlock.h>
#include <yarp/sig/Vector.h>

#include <BipedalLocomotion/System/VariablesHandler.h>

#include <fstream>
#include <iostream>
#include <thread>

std::ofstream myfile;

Eigen::Vector3d comPos0;
std::size_t indexSin{0};

using namespace CentroidalMPCWalking;
using namespace BipedalLocomotion::ParametersHandler;

constexpr double m_dT{0.001};

bool WholeBodyQPBlock::createKinDyn(const std::string& modelPath,
                                    const std::vector<std::string>& jointLists)
{
    constexpr auto errorPrefix = "[WholeBodyQPBlock::initializeLeggedOdometry]";

    auto handlerKinDyn = std::make_shared<StdImplementation>();
    handlerKinDyn->setParameter("joints_list", jointLists);
    handlerKinDyn->setParameter("model_file_name", modelPath);

    m_kinDynWithDesired
        = BipedalLocomotion::Estimators::constructKinDynComputationsDescriptor(handlerKinDyn);

    m_kinDynWithMeasured
        = BipedalLocomotion::Estimators::constructKinDynComputationsDescriptor(handlerKinDyn);

    if (!m_kinDynWithDesired.isValid() || !m_kinDynWithMeasured.isValid())
    {
        BipedalLocomotion::log()->error("{} Unable to load a KinDynComputation object",
                                        errorPrefix);
        return false;
    }

    return true;
}

bool WholeBodyQPBlock::instantiateIK(std::shared_ptr<const IParametersHandler> handler)
{
    using namespace BipedalLocomotion::IK;
    constexpr auto logError = "[WholeBodyQPBlock::instantiateIK]";

    Eigen::VectorXd weightRegularization(m_kinDynWithDesired.kindyn->getNrOfDegreesOfFreedom());

    handler->getGroup("REGULARIZATION_TASK").lock()->getParameter("weight", weightRegularization);


    Eigen::Vector3d weightChest;

    handler->getGroup("CHEST_TASK").lock()->getParameter("weight", weightChest);


    BipedalLocomotion::System::VariablesHandler variables;
    variables.addVariable("robotVelocity",
                          m_kinDynWithDesired.kindyn->getNrOfDegreesOfFreedom() + 6);

    constexpr std::size_t highPriority = 0;
    constexpr std::size_t lowPriority = 1;

    m_IKandTasks.ik = std::make_shared<QPInverseKinematics>();
    if (!m_IKandTasks.ik->initialize(handler))
    {
        BipedalLocomotion::log()->error("{} Unable to initialize the IK.", logError);
        return false;
    }

    m_IKandTasks.leftFootTask = std::make_shared<SE3Task>();
    if (!m_IKandTasks.leftFootTask->setKinDyn(m_kinDynWithDesired.kindyn))
    {
        BipedalLocomotion::log()->error("{} Unable to set the kinDynObject in the left foot.",
                                        logError);
        return false;
    }
    if (!m_IKandTasks.leftFootTask->initialize(handler->getGroup("L_FOOT")))
    {
        BipedalLocomotion::log()->error("{} Unable initialize in the left foot.", logError);
        return false;
    }
    if (!m_IKandTasks.ik->addTask(m_IKandTasks.leftFootTask, "l_foot_task", highPriority))
    {
        BipedalLocomotion::log()->error("{} Unable to add the left foot task.", logError);
        return false;
    }

    m_IKandTasks.rightFootTask = std::make_shared<SE3Task>();
    if (!m_IKandTasks.rightFootTask->setKinDyn(m_kinDynWithDesired.kindyn))
    {
        BipedalLocomotion::log()->error("{} Unable to set the kinDynObject in the right foot.",
                                        logError);
        return false;
    }
    if (!m_IKandTasks.rightFootTask->initialize(handler->getGroup("R_FOOT")))
    {
        BipedalLocomotion::log()->error("{} Unable initialize in the right foot.", logError);
        return false;
    }
    if (!m_IKandTasks.ik->addTask(m_IKandTasks.rightFootTask, "r_foot_task", highPriority))
    {
        BipedalLocomotion::log()->error("{} Unable to add the right foot task.", logError);
        return false;
    }

    //// com
    m_IKandTasks.comTask = std::make_shared<CoMTask>();
    if (!m_IKandTasks.comTask->setKinDyn(m_kinDynWithDesired.kindyn))
    {
        BipedalLocomotion::log()->error("{} Unable to set the kinDynObject for the com task.",
                                        logError);
        return false;
    }
    if (!m_IKandTasks.comTask->initialize(handler->getGroup("COM_TASK")))
    {
        BipedalLocomotion::log()->error("{} Unable initialize in the com task.", logError);
        return false;
    }
    if (!m_IKandTasks.ik->addTask(m_IKandTasks.comTask, "com", highPriority))
    {
        BipedalLocomotion::log()->error("{} Unable to add the com task.", logError);
        return false;
    }

    //// CHEST
    m_IKandTasks.chestTask = std::make_shared<SO3Task>();
    if (!m_IKandTasks.chestTask->setKinDyn(m_kinDynWithDesired.kindyn))
    {
        BipedalLocomotion::log()->error("{} Unable to set the kinDynObject for the com task.",
                                        logError);
        return false;
    }
    if (!m_IKandTasks.chestTask->initialize(handler->getGroup("CHEST_TASK")))
    {
        BipedalLocomotion::log()->error("{} Unable initialize in the com task.", logError);
        return false;
    }
    if (!m_IKandTasks.ik->addTask(m_IKandTasks.chestTask, "chest", lowPriority, weightChest))
    {
        BipedalLocomotion::log()->error("{} Unable to add the com task.", logError);
        return false;
    }

    m_IKandTasks.regularizationTask = std::make_shared<JointTrackingTask>();

    if (!m_IKandTasks.regularizationTask->setKinDyn(m_kinDynWithMeasured.kindyn))
    {
        BipedalLocomotion::log()->error("{} Unable to set the kinDynObject for the com task.",
                                        logError);
        return false;
    }
    if (!m_IKandTasks.regularizationTask->initialize(handler->getGroup("REGULARIZATION_TASK")))
    {
        BipedalLocomotion::log()->error("{} Unable initialize in the com task.", logError);
        return false;
    }

    if (!m_IKandTasks.ik->addTask(m_IKandTasks.regularizationTask,
                                  "regularization",
                                  lowPriority,
                                  weightRegularization))
    {
        BipedalLocomotion::log()->error("{} Unable to add the com task.", logError);
        return false;
    }

    if (!m_IKandTasks.ik->finalize(variables))
    {
        BipedalLocomotion::log()->error("{} Unable to finalize the IK.", logError);
        return false;
    }

    return true;
}

bool WholeBodyQPBlock::initializeRobotControl(std::shared_ptr<const IParametersHandler> handler)
{
    constexpr auto errorPrefix = "[WholeBodyQPBlock::initializeRobotControl]";

    if (!m_robotControl.initialize(handler->getGroup("ROBOT_CONTROL")))
    {
        BipedalLocomotion::log()->error("{} Unable to initialize the robot control.", errorPrefix);
        return false;
    }
    if (!m_robotControl.setDriver(m_controlBoard.poly))
    {
        BipedalLocomotion::log()->error("{} Unable to set the polydriver.", errorPrefix);
        return false;
    }

    return true;
}

bool WholeBodyQPBlock::instantiateSensorBridge(std::shared_ptr<const IParametersHandler> handler)
{
    constexpr auto errorPrefix = "[WholeBodyQPBlock::instantiateSensorBridge]";

    if (!m_sensorBridge.initialize(handler->getGroup("SENSOR_BRIDGE")))
    {
        BipedalLocomotion::log()->error("{} Unable to initialize the sensor bridge.", errorPrefix);
        return false;
    }

    yarp::dev::PolyDriverList list;
    list.push(m_controlBoard.poly.get(), m_controlBoard.key.c_str());
    if (!m_sensorBridge.setDriversList(list))
    {
        BipedalLocomotion::log()->error("{} Unable to set the driver list.", errorPrefix);
        return false;
    }

    return true;
}

bool WholeBodyQPBlock::createPolydriver(std::shared_ptr<const IParametersHandler> handler)
{
    constexpr auto errorPrefix = "[WholeBodyQPBlock::createPolydriver]";

    std::string name;
    if (!handler->getParameter("name", name))
    {
        BipedalLocomotion::log()->error("{} Unable to find the name.", errorPrefix);
        return false;
    }

    auto ptr = handler->getGroup("ROBOT_INTERFACE").lock();
    if (ptr == nullptr)
    {
        BipedalLocomotion::log()->error("{} Robot interface options is empty.", errorPrefix);
        return false;
    }
    ptr->setParameter("local_prefix", name);
    m_controlBoard = BipedalLocomotion::RobotInterface::constructRemoteControlBoardRemapper(ptr);
    if (!m_controlBoard.isValid())
    {
        BipedalLocomotion::log()->error("{} the robot polydriver has not been constructed.",
                                        errorPrefix);
        return false;
    }

    return true;
}

bool WholeBodyQPBlock::initialize(std::weak_ptr<const IParametersHandler> handler)
{
    constexpr auto logPrefix = "[WholeBodyQPBlock::initialize]";

    auto parametersHandler = handler.lock();

    BipedalLocomotion::log()->info("{} Create the polydriver.", logPrefix);
    if (!this->createPolydriver(parametersHandler))
    {
        BipedalLocomotion::log()->error("{} Unable to create the polydriver.", logPrefix);
        return false;
    }

    BipedalLocomotion::log()->info("{} Create the robot control helper.", logPrefix);
    if (!this->initializeRobotControl(parametersHandler))
    {
        BipedalLocomotion::log()->error("{} Unable to initialize the robotControl interface.",
                                        logPrefix);
        return false;
    }

    BipedalLocomotion::log()->info("{} Create the sensor bridge.", logPrefix);
    if (!this->instantiateSensorBridge(parametersHandler))
    {
        BipedalLocomotion::log()->error("{} Unable to initialize the sensor bridge.", logPrefix);
        return false;
    }

    std::string name;
    if (!parametersHandler->getParameter("name", name))
    {
        BipedalLocomotion::log()->error("{} Unable to find the name.", logPrefix);
        return false;
    }

    const std::string portRobotBase = "/" + name + "/robotBase:i";
    m_robotBasePort.open(portRobotBase);

    yarp::os::Network::connect("/icubSim/floating_base/state:o", portRobotBase);

    if (!this->createKinDyn(yarp::os::ResourceFinder::getResourceFinderSingleton().findFileByName(
                                "model.urdf"),
                            m_robotControl.getJointList()))
    {
        BipedalLocomotion::log()->error("{} Unable to initialize the kinDyn.", logPrefix);
        return false;
    }

    m_kinDynWithDesired.kindyn->setFloatingBase("root_link");
    m_kinDynWithMeasured.kindyn->setFloatingBase("root_link");

    if (!this->instantiateIK(parametersHandler->getGroup("IK").lock()))
    {
        BipedalLocomotion::log()->error("{} Unable to initialize the IK.", logPrefix);
        return false;
    }

    ///
    // Reset kinDynObject and floating base integrator
    const auto numberOfJoints = m_robotControl.getJointList().size();

    m_currentJointPos.resize(numberOfJoints);
    m_currentJointVel.resize(numberOfJoints);
    m_currentJointVel.setZero();
    m_desJointPos.resize(numberOfJoints);
    m_desJointVel.resize(numberOfJoints);
    m_desJointVel.setZero();

    if (!m_sensorBridge.advance())
    {
        BipedalLocomotion::log()->error("{} Unable to get the robot state.", logPrefix);
        return false;
    }
    m_sensorBridge.getJointPositions(m_currentJointPos);
    m_sensorBridge.getJointPositions(m_desJointPos);

    unsigned counter = 0;
    constexpr unsigned maxIter = 100;
    constexpr unsigned timeout = 500;
    yarp::sig::Vector* base = NULL;
    while (base == nullptr)
    {
        base = m_robotBasePort.read(false);
        if (++counter == maxIter)
        {
            BipedalLocomotion::log()->error("{} Unable to read the base pose.", logPrefix);
            return false;
        }

        // Sleep for some while
        std::this_thread::sleep_for(std::chrono::microseconds(timeout));
    }

    // TODO remove me
    Eigen::Vector3d translation = Eigen::Map<Eigen::Vector3d>(base->data(), 3);
    translation(2) -= 0.0105;

    m_baseTransform.translation(translation);
    m_baseTransform.quat(Eigen::AngleAxisd((*base)(5), Eigen::Vector3d::UnitZ())
                         * Eigen::AngleAxisd((*base)(4), Eigen::Vector3d::UnitY())
                         * Eigen::AngleAxisd((*base)(3), Eigen::Vector3d::UnitX()));

    m_baseVelocity.coeffs() = Eigen::Map<Eigen::Matrix<double, 6, 1>>(base->data() + 6, 6);

    Eigen::Vector3d gravity;
    gravity.setZero();

    m_kinDynWithDesired.kindyn->setRobotState(m_baseTransform.transform(),
                                              m_desJointPos,
                                              iDynTree::make_span(m_baseVelocity.data(),
                                                                  manif::SE3d::Tangent::DoF),
                                              m_desJointVel,
                                              gravity);

    constexpr double scaling = 0.5;
    constexpr double scalingPos = 0.0;
    // t  0   1   2   3   4   5   6   7   8   9   10  11  12  13  14  15  16  17
    // L |+++|---|+++++++++++|---|+++++++++++|---|+++++++++++|---|+++++++++++|
    // R |+++++++++++|---|+++++++++++|---|+++++++++++|---|+++++++++++|---|+++|
    BipedalLocomotion::Contacts::ContactListMap contactListMap;

    manif::SE3d leftTransform(BipedalLocomotion::Conversions::toManifPose(
        m_kinDynWithDesired.kindyn->getWorldTransform("l_sole")));
    Eigen::Vector3d leftPosition = leftTransform.translation();
    contactListMap["left_foot"].addContact(leftTransform, 0.0, 1.0 * scaling);

    leftPosition(0) += 0.05 * scalingPos;
    leftTransform.translation(leftPosition);
    contactListMap["left_foot"].addContact(leftTransform, 2.0 * scaling, 5.0 * scaling);

    leftPosition(0) += 0.1 * scalingPos;
    leftTransform.translation(leftPosition);
    contactListMap["left_foot"].addContact(leftTransform, 6.0 * scaling, 9.0 * scaling);

    leftPosition(0) += 0.1 * scalingPos;
    leftTransform.translation(leftPosition);
    contactListMap["left_foot"].addContact(leftTransform, 10.0 * scaling, 13.0 * scaling);

    leftPosition(0) += 0.1 * scalingPos;
    leftTransform.translation(leftPosition);
    contactListMap["left_foot"].addContact(leftTransform, 14.0 * scaling, 17.0 * scaling);

    // right foot
    // first footstep
    manif::SE3d rightTransform(BipedalLocomotion::Conversions::toManifPose(
        m_kinDynWithDesired.kindyn->getWorldTransform("r_sole")));

    Eigen::Vector3d rightPosition = rightTransform.translation();

    BipedalLocomotion::log()->info("Right transform {}", rightTransform);

    contactListMap["right_foot"].addContact(rightTransform, 0.0, 3.0 * scaling);

    rightPosition(0) += 0.1 * scalingPos;
    rightTransform.translation(rightPosition);
    contactListMap["right_foot"].addContact(rightTransform, 4.0 * scaling, 7.0 * scaling);

    rightPosition(0) += 0.1 * scalingPos;
    rightTransform.translation(rightPosition);
    contactListMap["right_foot"].addContact(rightTransform, 8.0 * scaling, 11.0 * scaling);

    rightPosition(0) += 0.1 * scalingPos;
    rightTransform.translation(rightPosition);
    contactListMap["right_foot"].addContact(rightTransform, 12.0 * scaling, 15.0 * scaling);

    rightPosition(0) += 0.05 * scalingPos;
    rightTransform.translation(rightPosition);
    contactListMap["right_foot"].addContact(rightTransform, 16.0 * scaling, 17.0 * scaling);

    // feet

    std::shared_ptr<IParametersHandler> a = std::make_shared<StdImplementation>();
    a->setParameter("sampling_time", m_dT);
    a->setParameter("step_height", 0.02);
    a->setParameter("foot_apex_time", 0.5);
    a->setParameter("foot_landing_velocity", 0.0);
    a->setParameter("foot_landing_acceleration", 0.0);

    m_leftFootPlanner.initialize(a);
    m_rightFootPlanner.initialize(a);

    m_leftFootPlanner.setContactList(contactListMap["left_foot"]);
    m_rightFootPlanner.setContactList(contactListMap["right_foot"]);

    BipedalLocomotion::log()->info("COM-------------_> {}",
                                   m_kinDynWithDesired.kindyn->getCenterOfMassPosition().toString());

    BipedalLocomotion::log()
        ->info("l_sole-------------_> {}",
               m_kinDynWithDesired.kindyn->getWorldTransform("l_sole").toString());

    BipedalLocomotion::log()
        ->info("r_sole-------------_> {}",
               m_kinDynWithDesired.kindyn->getWorldTransform("r_sole").toString());

    comPos0 = iDynTree::toEigen(m_kinDynWithDesired.kindyn->getCenterOfMassPosition());

    m_IKandTasks.regularizationTask->setSetPoint(m_desJointPos);

    using namespace BipedalLocomotion::ContinuousDynamicalSystem;
    m_system.dynamics = std::make_shared<FloatingBaseSystemKinematics>();
    m_system.dynamics->setState({m_baseTransform.transform().topRightCorner<3, 1>(),
                                 m_baseTransform.transform().topLeftCorner<3, 3>(),
                                 m_desJointPos});

    m_system.integrator = std::make_shared<ForwardEuler<FloatingBaseSystemKinematics>>();
    m_system.integrator->setIntegrationStep(m_dT);
    m_system.integrator->setDynamicalSystem(m_system.dynamics);

    ////// centroidal system
    m_centroidalSystem.dynamics = std::make_shared<CentroidalDynamics>();
    m_centroidalSystem.dynamics->setState(
        {comPos0, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()});

    m_centroidalSystem.integrator = std::make_shared<ForwardEuler<CentroidalDynamics>>();
    m_centroidalSystem.integrator->setIntegrationStep(m_dT);
    m_centroidalSystem.integrator->setDynamicalSystem(m_centroidalSystem.dynamics);

    myfile.open("example.txt");

    return true;
}

const WholeBodyQPBlock::Output& WholeBodyQPBlock::getOutput() const
{
    return m_output;
}

bool WholeBodyQPBlock::setInput(const Input& input)
{
    m_input = input;
    return true;
}

bool WholeBodyQPBlock::advance()
{
    bool shouldAdvance = false;

    if (m_input.contacts.size() != 0)
    {
        Eigen::Vector3d force = Eigen::Vector3d::Zero();
        for (const auto& [key, value] : m_input.contacts)
        {
            for (const auto& c : value.corners)
            {
                force += c.force;
                // BipedalLocomotion::log()->info(" Foot {}, force: {}", key, c.force.transpose());
            }
        }

        if (force.norm() > 0.01)
        {
            m_centroidalSystem.dynamics->setControlInput(m_input.contacts);
            m_centroidalSystem.integrator->integrate(0, m_dT);
        }

        shouldAdvance = true;
    }

    Eigen::Vector3d gravity;
    gravity.setZero();
    gravity(2) = -BipedalLocomotion::Math::StandardAccelerationOfGravitation;

    auto startTotal = std::chrono::steady_clock::now();
    auto start1 = std::chrono::steady_clock::now();
    constexpr auto errorPrefix = "[WholeBodyQPBlock::advance]";
    if (!m_sensorBridge.advance())
    {
        BipedalLocomotion::log()->error("{} Unable to get the robot state.", errorPrefix);
        return false;
    }

    m_sensorBridge.getJointPositions(m_currentJointPos);
    m_sensorBridge.getJointVelocities(m_currentJointVel);

    auto end1 = std::chrono::steady_clock::now();
    std::chrono::duration<double, std::milli> elapsed1 = end1 - start1;
    // BipedalLocomotion::log()->info("sensor bridge - time required {}  ms", (elapsed1).count());

    start1 = std::chrono::steady_clock::now();

    unsigned counter = 0;
    constexpr unsigned maxIter = 100;
    constexpr unsigned timeout = 5;
    yarp::sig::Vector* base = NULL;
    while (base == nullptr)
    {
        base = m_robotBasePort.read(false);
        if (++counter == maxIter)
        {
            BipedalLocomotion::log()->error("{} Unable to read the base pose.", errorPrefix);
            return false;
        }

        // Sleep for some while
        std::this_thread::sleep_for(std::chrono::microseconds(timeout));
    }

    end1 = std::chrono::steady_clock::now();
    elapsed1 = end1 - start1;
    // BipedalLocomotion::log()->info("base - time required {}  ms", (elapsed1).count());

    m_kinDynWithMeasured.kindyn->setRobotState(m_baseTransform.transform(),
                                               m_currentJointPos,
                                               iDynTree::make_span(m_baseVelocity.data(),
                                                                   manif::SE3d::Tangent::DoF),
                                               m_currentJointVel,
                                               gravity);

    // TODO remove me
    Eigen::Vector3d translation = Eigen::Map<Eigen::Vector3d>(base->data(), 3);
    translation(2) -= 0.0105;

    m_baseTransform.translation(translation);
    m_baseTransform.quat(Eigen::AngleAxisd((*base)(5), Eigen::Vector3d::UnitZ())
                         * Eigen::AngleAxisd((*base)(4), Eigen::Vector3d::UnitY())
                         * Eigen::AngleAxisd((*base)(3), Eigen::Vector3d::UnitX()));

    m_baseVelocity.coeffs() = Eigen::Map<Eigen::Matrix<double, 6, 1>>(base->data() + 6, 6);

    auto start = std::chrono::steady_clock::now();

    // if (!m_leftFootPlanner.getOutput().mixedVelocity.coeffs().isZero())
    // {
    //     m_IKandTasks.leftFootTask->enableControl();
    // } else
    // {
    //     m_IKandTasks.leftFootTask->disableControl();
    // }

    // if (!m_rightFootPlanner.getOutput().mixedVelocity.coeffs().isZero())
    // {
    //     m_IKandTasks.rightFootTask->enableControl();
    // } else
    // {
    //     m_IKandTasks.rightFootTask->disableControl();
    // }

    m_IKandTasks.leftFootTask->setSetPoint(m_leftFootPlanner.getOutput().transform,
                                           m_leftFootPlanner.getOutput().mixedVelocity);

    m_IKandTasks.rightFootTask->setSetPoint(m_rightFootPlanner.getOutput().transform,
                                            m_rightFootPlanner.getOutput().mixedVelocity);

    m_IKandTasks.comTask->setSetPoint(std::get<0>(m_centroidalSystem.dynamics->getState()),
                                      std::get<1>(m_centroidalSystem.dynamics->getState()));

    double yawLeft
        = m_leftFootPlanner.getOutput().transform.quat().toRotationMatrix().eulerAngles(2, 1, 0)(0);

    double yawRight
        = m_rightFootPlanner.getOutput().transform.quat().toRotationMatrix().eulerAngles(2, 1, 0)(
            0);

    const double meanYaw = std::atan2(std::sin(yawLeft) + std::sin(yawRight),
                                      std::cos(yawLeft) + std::cos(yawRight));

    manif::SO3d chestOrientation = manif::SO3d(Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitX()));


        // ((0.0 0.0 1.0),
        //  (1.0 0.0 0.0),
        //  (0.0 1.0 0.0));

    m_IKandTasks.chestTask->setSetPoint(chestOrientation, manif::SO3d::Tangent::Zero());

    if (!m_IKandTasks.ik->advance())
    {
        BipedalLocomotion::log()->error("{} Unable to solve the IK problem", errorPrefix);
        return false;
    }
    auto end = std::chrono::steady_clock::now();

    if (shouldAdvance)
    {
        m_leftFootPlanner.advance();
        m_rightFootPlanner.advance();
    }
    std::chrono::duration<double, std::milli> elapsed = end - start;

    // BipedalLocomotion::log()->info("IK - time required {}  ms", (elapsed).count());

    // double freq = 0.2;
    // comPos(1)+= 0.07 * std::sin(2 * M_PI * freq * indexSin * m_dT);
    // Eigen::Vector3d comVel = Eigen::Vector3d::Zero();
    // comVel(1) = 2 * M_PI * freq * 0.07 * std::cos(2 * M_PI * freq * indexSin * m_dT);

    start = std::chrono::steady_clock::now();

    m_system.dynamics->setControlInput({m_IKandTasks.ik->getOutput().baseVelocity.coeffs(),
                                        m_IKandTasks.ik->getOutput().jointVelocity});
    m_system.integrator->integrate(0, m_dT);

    const auto& [basePosition, baseRotation, jointPosition] = m_system.integrator->getSolution();

    end = std::chrono::steady_clock::now();
    elapsed = end - start;
    // BipedalLocomotion::log()->info("Integrator - time required {}  ms", (elapsed).count());

    // start =  std::chrono::steady_clock::now();

    // end =  std::chrono::steady_clock::now();
    // elapsed = end-start;
    // BipedalLocomotion::log()->info("Set reference - time required {}  ms", (elapsed).count());

    /////// update kinDyn
    m_kinDynWithDesired.kindyn->setRobotState(m_baseTransform.transform(),
                                              jointPosition,
                                              iDynTree::make_span(m_baseVelocity.data(),
                                                                  manif::SE3d::Tangent::DoF),
                                              m_IKandTasks.ik->getOutput().jointVelocity,
                                              gravity);

    // BipedalLocomotion::log()->info("Total - time required {}  ms", (elapsedTotal).count());

    // m_kinDynWithMeasured.kindyn->getCenterOfMassPosition(m_output.com);
    // m_kinDynWithMeasured.kindyn->getCenterOfMassVelocity(m_output.dcom);
    // m_output.angularMomentum
    //     = iDynTree::toEigen(
    //         m_kinDynWithMeasured.kindyn->getCentroidalTotalMomentum().getAngularVec3());

    m_output.leftFoot = BipedalLocomotion::Conversions::toManifPose(
        m_kinDynWithMeasured.kindyn->getWorldTransform("l_sole"));

    m_output.rightFoot = BipedalLocomotion::Conversions::toManifPose(
        m_kinDynWithMeasured.kindyn->getWorldTransform("r_sole"));

    // TODO remove me
    m_output.com = std::get<0>(m_centroidalSystem.dynamics->getState());
    m_output.dcom = std::get<1>(m_centroidalSystem.dynamics->getState());
    m_output.angularMomentum = std::get<2>(m_centroidalSystem.dynamics->getState());

    m_output.isValid = true;

    // myfile
    //     << m_kinDynWithMeasured.kindyn->getCenterOfMassPosition().toString() << " "
    //     << std::get<0>(m_centroidalSystem.dynamics->getState()).transpose() << " "
    //     << m_leftFootPlanner.getOutput().transform.translation().transpose() << " "
    //     << iDynTree::toEigen(m_kinDynWithMeasured.kindyn->getWorldTransform("l_sole").getPosition())
    //            .transpose()
    //     << " "
    //     << iDynTree::toEigen(m_kinDynWithDesired.kindyn->getWorldTransform("l_sole").getPosition())
    //     .transpose() << " " << jointPosition.transpose() << " " <<  m_currentJointPos.transpose()
    //     << std::endl;

    myfile
        << jointPosition.transpose() << " " << m_currentJointPos.transpose() << " "
        << m_kinDynWithMeasured.kindyn->getCenterOfMassPosition().toString() << " "
        << std::get<0>(m_centroidalSystem.dynamics->getState()).transpose() << " "
        << m_leftFootPlanner.getOutput().transform.translation().transpose() << " "
        << iDynTree::toEigen(m_kinDynWithMeasured.kindyn->getWorldTransform("l_sole").getPosition())
               .transpose()
        << " "
        << iDynTree::toEigen(m_kinDynWithDesired.kindyn->getWorldTransform("l_sole").getPosition())
               .transpose()
        << std::endl;

    if (!m_robotControl.setReferences(jointPosition,
                                      BipedalLocomotion::RobotInterface::IRobotControl::
                                          ControlMode::PositionDirect))
    {
        BipedalLocomotion::log()->error("{} Unable to set the reference", errorPrefix);
        return false;
    }

    return true;
}

bool WholeBodyQPBlock::isOutputValid() const
{
    return true;
}

bool WholeBodyQPBlock::close()
{
    return true;
}
