/**
 * @file CentroidalMPCBlock.cpp
 * @authors Giulio Romualdi
 * @copyright This software may be modified and distributed under the terms of the GNU Lesser
 * General Public License v2.1 or any later version.
 */

#include <BipedalLocomotion/Contacts/ContactPhaseList.h>
#include <BipedalLocomotion/Planners/QuinticSpline.h>

#include <CentroidalMPCWalking/CentroidalMPCBlock.h>
#include <manif/manif.h>

#include <BipedalLocomotion/TextLogging/Logger.h>

using namespace CentroidalMPCWalking;

bool isFirstRun{true};
Eigen::MatrixXd comTraj(3, 500);
int indexCoM{0};
bool CentroidalMPCBlock::initialize(
    std::weak_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> handler)
{
    if (!m_controller.initialize(handler))
    {
        BipedalLocomotion::log()->error("[CentroidalMPCBlock::initialize] Unable to initialize the "
                                        "controller");
        return false;
    }

    return true;
}

const CentroidalMPCBlock::Output& CentroidalMPCBlock::getOutput() const
{
    return m_controller.getOutput();
}

bool CentroidalMPCBlock::setInput(const Input& input)
{
    if(input.isValid)
    {
        m_inputValid = input.isValid;

        if(isFirstRun)
        {
            isFirstRun = false;

            // TODO Remove me!!!
            // left foot
            // first footstep

    constexpr double scaling = 0.5;
    constexpr double scalingPos = 0.0;
            // t  0   1   2   3   4   5   6   7   8   9   10  11  12  13  14  15  16  17
            // L |+++|---|+++++++++++|---|+++++++++++|---|+++++++++++|---|+++++++++++|
            // R |+++++++++++|---|+++++++++++|---|+++++++++++|---|+++++++++++|---|+++|
            BipedalLocomotion::Contacts::ContactListMap contactListMap;

            Eigen::Vector3d leftPosition = input.leftFoot.translation();
            manif::SE3d leftTransform(leftPosition, manif::SO3d::Identity());
            contactListMap["left_foot"].addContact(leftTransform, 0.0, 1.0 * scaling);

            leftPosition(0) += 0.05 * scalingPos;
            leftTransform.translation(leftPosition);
            contactListMap["left_foot"].addContact(leftTransform, 2.0* scaling, 5.0* scaling);

            leftPosition(0) += 0.1 * scalingPos;
            leftTransform.translation(leftPosition);
            contactListMap["left_foot"].addContact(leftTransform, 6.0* scaling, 9.0* scaling);

            leftPosition(0) += 0.1 * scalingPos;
            leftTransform.translation(leftPosition);
            contactListMap["left_foot"].addContact(leftTransform, 10.0* scaling, 13.0* scaling);

            leftPosition(0) += 0.1 * scalingPos;
            leftTransform.translation(leftPosition);
            contactListMap["left_foot"].addContact(leftTransform, 14.0* scaling, 17.0* scaling);

            // right foot
            // first footstep
            Eigen::Vector3d rightPosition = input.rightFoot.translation();
            manif::SE3d rightTransform(rightPosition, manif::SO3d::Identity());

            contactListMap["right_foot"].addContact(rightTransform, 0.0, 3.0* scaling);

            rightPosition(0) += 0.1 * scalingPos;
            rightTransform.translation(rightPosition);
            contactListMap["right_foot"].addContact(rightTransform, 4.0* scaling, 7.0* scaling);

            rightPosition(0) += 0.1 * scalingPos;
            rightTransform.translation(rightPosition);
            contactListMap["right_foot"].addContact(rightTransform, 8.0* scaling, 11.0* scaling);

            rightPosition(0) += 0.1 * scalingPos;
            rightTransform.translation(rightPosition);
            contactListMap["right_foot"].addContact(rightTransform, 12.0* scaling, 15.0* scaling);

            rightPosition(0) += 0.05 * scalingPos;
            rightTransform.translation(rightPosition);
            contactListMap["right_foot"].addContact(rightTransform, 16.0* scaling, 17.0* scaling);
            m_phaseList.setLists(contactListMap);

            BipedalLocomotion::log()->info("CoM {}", input.com);

            double CoMOffsetX
                = input.com(0)
                - (input.rightFoot.translation()(0) + input.leftFoot.translation()(0)) / 2;

            std::vector<Eigen::VectorXd> comKnots;
            std::vector<double> timeKnots;

            BipedalLocomotion::Planners::QuinticSpline comSpline;
            timeKnots.push_back(m_phaseList.cbegin()->beginTime);
            comKnots.push_back(input.com);
            for (auto it = m_phaseList.begin(); it != m_phaseList.end(); std::advance(it, 1))
            {
                if (it->activeContacts.size() == 2 && it != m_phaseList.begin()
                    && it != m_phaseList.lastPhase())
                {
                    timeKnots.emplace_back((it->endTime + it->beginTime) / 2);

                    auto contactIt = it->activeContacts.cbegin();
                    const Eigen::Vector3d p1 = contactIt->second->pose.translation();
                    std::advance(contactIt, 1);
                    const Eigen::Vector3d p2 = contactIt->second->pose.translation();

                    Eigen::Vector3d desiredCoMPosition = (p1 + p2) / 2.0;
                    desiredCoMPosition(2) += input.com(2);
                    desiredCoMPosition(0) += CoMOffsetX;

                    comKnots.emplace_back(desiredCoMPosition);
                }

                else if (it->activeContacts.size() == 2 && it == m_phaseList.lastPhase())
                {
                    timeKnots.push_back(it->endTime);
                    auto contactIt = it->activeContacts.cbegin();
                    const Eigen::Vector3d p1 = contactIt->second->pose.translation();
                    std::advance(contactIt, 1);
                    const Eigen::Vector3d p2 = contactIt->second->pose.translation();

                    Eigen::Vector3d desiredCoMPosition = (p1 + p2) / 2.0;
                    desiredCoMPosition(2) += input.com(2);
                    desiredCoMPosition(0) += CoMOffsetX ;

                    comKnots.emplace_back(desiredCoMPosition);
                }
            }


            comSpline.setInitialConditions(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
            comSpline.setFinalConditions(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
            comSpline.setKnots(comKnots, timeKnots);


            Eigen::Vector3d velocity, acceleration;

            for(int i = 0; i < 170 / scaling; i++)
            {
                // TODO remove me
                comSpline.evaluatePoint(i * 0.1,
                                        comTraj.col(i),
                                        velocity,
                                        acceleration);
            }

            // i = 3
            // * *
            // 1 2 3 4 5
            comTraj.rightCols(500 - 169).colwise() = comTraj.col(169);
        }

        return m_controller.setState(input.com, input.dcom, input.angularMomentum);
    }
    return true;
}

bool CentroidalMPCBlock::advance()
{
    if (m_inputValid)
    {
        m_controller.setReferenceTrajectory(comTraj.rightCols(comTraj.cols() - indexCoM));
        m_controller.setContactPhaseList(m_phaseList);
        bool ok = m_controller.advance();
        indexCoM++;
        return ok;

    }
    return true;
}

bool CentroidalMPCBlock::isOutputValid() const
{
    return m_controller.isOutputValid();
}
