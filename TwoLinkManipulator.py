import numpy as np
import matplotlib.pyplot as plt

class TwoLinkManipulator():
    '''
    Class to simulate a two-link planar robotic manipulator.

    Attributes:
        L1 (float): Length of the first link.
        L2 (float): Length of the second link.
        theta1 (float): Rotation angle of the first joint in radians.
        theta2 (float): Rotation angle of the second joint in radians.
    '''
    def __init__(self, L1, L2, theta1, theta2):
        '''
        Initializes the TwoLinkManipulator object.

        Parameters:
            L1 (float): Length of the first link.
            L2 (float): Length of the second link.
            theta1 (float): Rotation angle of the first joint in radians.
            theta2 (float): Rotation angle of the second joint in radians.

        Raises:
            ValueError: If L1 or L2 is non-positive.
        '''
        if L1 <= 0 or L2 <= 0:
            raise ValueError('Link Lengths Must be positive')
        self.L1 = L1
        self.L2 = L2
        self.theta1 = theta1
        self.theta2 = theta2

    def calculate_transformation_matrices(self, theta1, theta2):
        '''
        Calculates the transformation matrices for the manipulator's links.

        Parameters:
            theta1 (float): Rotation angle of the first joint in radians.
            theta2 (float): Rotation angle of the second joint in radians.

        Returns:
            transformation matrices T02 and T01.
        '''
        A1 = np.array([
            [np.cos(theta1), -np.sin(theta1), 0, self.L1 * np.cos(theta1)],
            [np.sin(theta1), np.cos(theta1), 0, self.L1 * np.sin(theta1)],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])     

        A2 = np.array([
            [np.cos(theta2), -np.sin(theta2), 0, self.L2 * np.cos(theta2)],
            [np.sin(theta2), np.cos(theta2), 0, self.L2 * np.sin(theta2)],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])        

        try:
            T01 = A1
            T02 = np.dot(A1, A2)
        except ValueError as e:
            print(f"Matrix operation failed: {e}")
            return None, None
        return T02, T01
    
    def extract_positions(self):
        '''
        Extracts end effector and joint2 positions.

        Returns:
            tuple: A tuple containing the end effector positions and joint2 positions.
        '''
        L1 = self.L1
        L2 = self.L2

        theta1 = self.theta1
        theta2 = self.theta2

        num_points = 100

        theta1_values = np.linspace(0, theta1, num_points)
        theta2_values = np.linspace(0, theta2, num_points)

        #initialize emoty list to positions
        end_effector_positions = []
        joint2_position = []

        # calculate T01,T02 Transformation matrix for each theta values
        for thet1, thet2 in zip(theta1_values, theta2_values):
            T02, T01 = self.calculate_transformation_matrices(thet1, thet2)
            end_effector_positions.append((T02[0, 3], T02[1, 3]))
            joint2_position.append((T01[0, 3], T01[1, 3]))

        end_effector_positions = np.array(end_effector_positions)
        joint2_position = np.array(joint2_position)

        return end_effector_positions, joint2_position

    def motion_plot(self):
        '''
        Plots the motion of the manipulator.
        '''
        end_effector_positions, joint2_position = self.extract_positions()
        joint2_x = joint2_position[:, 0]
        joint2_y = joint2_position[:, 1]

        end_effector_x = end_effector_positions[:, 0]
        end_effector_y = end_effector_positions[:, 1]

        # Set up the figure and axes
        fig, ax = plt.subplots()

        ax.set_xlim([-(self.L1+self.L2)-1 , (self.L1+self.L2)+1])
        ax.set_ylim([-(self.L1+self.L2)-1 , (self.L1+self.L2)+1])
        ax.grid()
        ax.set_ylabel('Y')
        ax.set_xlabel('X')
        ax.set_title("Two Link Manipulator Motion")
        
        #refrence 
        square_vertices = np.array([[0.25,-0.25],[0.25,0.25],[-0.25,0.25],[-0.25,-0.25],[0.25,-0.25]])
        ax.fill(square_vertices[:, 0], square_vertices[:, 1],'-g')


        # Initialize empty lines for the links and joints
        link1, = ax.plot([], [], 'k-', lw=2)
        link2, = ax.plot([], [], 'k-', lw=2)
        joint1 = ax.plot([0], [0], 'bo', markersize=6)
        joint2, = ax.plot([], [], 'bo', markersize=6)
        end_efector, = ax.plot([], [], 'bx', markersize=6)

        #update links ,joints and end effector positions
        for i in range(len(end_effector_x)):
            link1.set_data([0, joint2_x[i]], [0, joint2_y[i]])
            link2.set_data([joint2_x[i], end_effector_x[i]], [joint2_y[i], end_effector_y[i]])
            joint2.set_data([joint2_x[i], joint2_y[i]])
            end_efector.set_data([end_effector_x[i], end_effector_y[i]])

            plt.pause(0.05)
        plt.show()
