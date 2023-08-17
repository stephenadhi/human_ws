
import numpy as np
import math
from soloco_interfaces.msg import TrackedPerson, TrackedPersons
from geometry_msgs.msg import Pose


class SFM():
    def __init__(self):

        self.forceFactorDesired = 2.0
        self.forceFactorObstacle = 10
        self.forceSigmaObstacle = 0.2
        self.forceFactorSocial = 2.1
        self.forceFactorGroupGaze = 3.0
        self.forceFactorGroupCoherence = 2.0
        self.forceFactorGroupRepulsion = 1.0
        self.lambda_ = 2.0
        self.gamma = 0.35
        self.n = 2.0
        self.nPrime = 3.0
        self.relaxationTime = 0.5
    
    def computeSocialForceTwoAgents(self, agent1, agent2):
        '''
        It computes the social force
        provoked by the agent1 in the
        agent2 
        '''
        socialForce = np.array([0.0, 0.0], dtype=np.float32)
        agent2pos = np.array([agent2.current_pose.pose.position.x, agent2.current_pose.pose.position.y])
        agent1pos = np.array([agent1.current_pose.pose.position.x, agent1.current_pose.pose.position.y])
        diff = agent1pos - agent2pos
        diffDirection = diff/np.linalg.norm(agent1pos - agent2pos)

        agent1_vel_x = (agent1.track.poses[-1].pose.position.x - agent1.track.poses[-2].pose.position.x) / 0.4
        agent1_vel_y = (agent1.track.poses[-1].pose.position.y - agent1.track.poses[-2].pose.position.y) / 0.4
        agent2_vel_x = (agent2.track.poses[-1].pose.position.x - agent2.track.poses[-2].pose.position.x) / 0.4
        agent2_vel_y = (agent2.track.poses[-1].pose.position.y - agent2.track.poses[-2].pose.position.y) / 0.4
        agent2vel = np.array([agent2_vel_x, agent2_vel_y])
        agent1vel = np.array([agent1_vel_x, agent1_vel_y])
        velDiff = agent2vel - agent1vel
        interactionVector = self.lambda_ * velDiff + diffDirection
        interactionLength = np.linalg.norm(interactionVector)
        interactionDirection = interactionVector / interactionLength
        theta = math.atan2(diffDirection[1], diffDirection[0]) - math.atan2(interactionDirection[0], interactionDirection[1])
        theta = self.normalize_angle(theta)
        B = self.gamma * interactionLength
        forceVelocityAmount = -math.exp(-np.linalg.norm(agent1pos - agent2pos) / B - (self.nPrime * B * theta)**2)
        forceAngleAmount = -np.sign(theta) * math.exp(-np.linalg.norm(agent1pos - agent2pos) / B - (self.n * B * theta)**2)
        forceVelocity = forceVelocityAmount * interactionDirection
        interactionDirection_leftNormal = np.array([-interactionDirection[1], interactionDirection[0]])
        forceAngle = forceAngleAmount * interactionDirection_leftNormal
        socialForce = self.forceFactorSocial * (forceVelocity + forceAngle)
        return socialForce


    def computeSocialForce(self, agent, agents):
        '''
        It computes the social force
        provoked by the agents in the
        agent 
        '''
        socialForce = np.array([0.0, 0.0], dtype=np.float32)
        for a in agents.tracks:
            if a.track_id == agent.track_id:
                continue
            # force provoked by agent 'a' in agent 'agent'
            socialForce += self.computeSocialForceTwoAgents(a, agent)
        return socialForce 


    def modulusSocialForce(self, agent, agents):
        '''
        It computes the cumulative modulus of 
        the social force provoked by the agents in the agent 
        '''
        socialForceMod = 0.0
        for a in agents.tracks:
            if a.track_id == agent.track_id:
                continue
            # force provoked by agent 'a' in agent 'agent'
            socialForceMod += np.linalg.norm(self.computeSocialForceTwoAgents(a, agent))
        return socialForceMod 


    def computeSocialForce2(self, agent, agents):
        '''
        It computes the social force
        provoked by the agent in the
        rest of agents 
        '''
        socialForce = np.array([0.0, 0.0], dtype=np.float32)
        for a in agents.tracks:
            if a.track_id == agent.track_id:
                continue
            # force provoked by agent 'agent' in agent 'a'
            socialForce += self.computeSocialForceTwoAgents(agent, a)
        return socialForce 
    

    def modulusSocialForce2(self, agent, agents):
        '''
        It computes the cumulative modulus 
        of the social force provoked by the agent 
        in the rest of agents 
        '''
        socialForceMod = 0.0
        for a in agents.tracks:
            # force provoked by agent 'agent' in agent 'a'
            socialForceMod += np.linalg.norm(self.computeSocialForceTwoAgents(agent, a))
        return socialForceMod 

    def normalize_angle(self, theta):
        # theta should be in the range [-pi, pi]
        if theta > math.pi:
            theta -= 2 * math.pi
        elif theta < -math.pi:
            theta += 2 * math.pi
        return theta