#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <cstdint>

#include <../include/VelocityManager.hpp>

struct VelocityManagerTest : public ::testing::Test
{
    using RobotStates = StateMachine<AcceptAllState, DeadManState, JoyState, AutonomousState>;
    RobotStates rs;
    VelocityManager vm;

    virtual void SetUp() override
    {
        printf("setup!!!!!!!!!\n");
        VelocityManager vm = VelocityManager();
    }
    virtual void TearDown()
    {
        printf("closing!!!!!!!\n");
    }
};

TEST_F(VelocityManagerTest, BasicAssertions)
{
    // Expect two strings not to be equal.
    EXPECT_STRNE("hello", "world");
    // Expect equality.
    EXPECT_EQ(7 * 6, 42);
}

TEST_F(VelocityManagerTest, TestSMTransition)
{
    // initial accept all state
    EXPECT_EQ(rs.getCStateIndex(), 0) << "Failed index don't match expected: 0 (AcceptAllState)  real value is: " << rs.getCStateIndex() << "\n";
    // test accept all internal loops
    rs.handle(AcceptAllEvent{});
    EXPECT_EQ(rs.getCStateIndex(), 0) << "Failed index don't match expected: 0 (AcceptAllState)  real value is: " << rs.getCStateIndex() << "\n";
    rs.handle(CmdEvent{});
    EXPECT_EQ(rs.getCStateIndex(), 0) << "Failed index don't match expected: 0 (AcceptAllState)  real value is: " << rs.getCStateIndex() << "\n";

    // going to DeadManState
    rs.handle(EnableDeadManEvent{});
    EXPECT_EQ(rs.getCStateIndex(), 1) << "Failed index don't match expected: 1 (DeadManState)  real value is: " << rs.getCStateIndex() << "\n";
    rs.handle(DeadManEvent{});
    EXPECT_EQ(rs.getCStateIndex(), 1) << "Failed index don't match expected: 1 (DeadManState)  real value is: " << rs.getCStateIndex() << "\n";
    rs.handle(AutonomousEvent{});
    EXPECT_EQ(rs.getCStateIndex(), 1) << "Failed index don't match expected: 1 (DeadManState)  real value is: " << rs.getCStateIndex() << "\n";
    // check ignored events
    rs.handle(JoyCmdEvent{});
    EXPECT_EQ(rs.getCStateIndex(), 1) << "Failed index don't match expected: 1 (DeadManState)  real value is: " << rs.getCStateIndex() << "\n";
    rs.handle(TimeoutEvent{});
    EXPECT_EQ(rs.getCStateIndex(), 1) << "Failed index don't match expected: 1 (DeadManState)  real value is: " << rs.getCStateIndex() << "\n";

    // going back to accept all
    rs.handle(AcceptAllEvent{});
    EXPECT_EQ(rs.getCStateIndex(), 0) << "Failed index don't match expected: 0 (AcceptAllState)  real value is: " << rs.getCStateIndex() << "\n";

    // transition to joy state by JoyCmdEvent
    rs.handle(JoyCmdEvent{});
    EXPECT_EQ(rs.getCStateIndex(), 2) << "Failed index don't match expected: 2 (JoyState)  real value is: " << rs.getCStateIndex() << "\n";
    // internal loop
    rs.handle(JoyCmdEvent{});
    EXPECT_EQ(rs.getCStateIndex(), 2) << "Failed index don't match expected: 2 (JoyState)  real value is: " << rs.getCStateIndex() << "\n";

    // going back to accept all
    rs.handle(AcceptAllEvent{});
    EXPECT_EQ(rs.getCStateIndex(), 0) << "Failed index don't match expected: 0 (AcceptAllState)  real value is: " << rs.getCStateIndex() << "\n";

    // Autonomous
    rs.handle(AutonomousEvent{});
    EXPECT_EQ(rs.getCStateIndex(), 3) << "Failed index don't match expected: 3 (AutonomousState)  real value is: " << rs.getCStateIndex() << "\n";
    rs.handle(AutonomousEvent{});
    EXPECT_EQ(rs.getCStateIndex(), 3) << "Failed index don't match expected: 3 (AutonomousState)  real value is: " << rs.getCStateIndex() << "\n";

    // go back to accept all
    rs.handle(TimeoutEvent{});
    EXPECT_EQ(rs.getCStateIndex(), 0) << "Failed index don't match expected: 0 (AcceptAllState)  real value is: " << rs.getCStateIndex() << "\n";
}