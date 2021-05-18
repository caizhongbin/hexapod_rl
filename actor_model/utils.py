import numpy as np
from mpl_toolkits import mplot3d
from matplotlib import pyplot as plt

# from robot_paras import robot_paras
from actor_model.robot_paras import robot_paras

class IK:
    def __init__(self):
        self.params = robot_paras()
        self.three_gait = [[1, -1, 1, -1, 1, -1], [-1, 1, -1, 1, -1, 1]]
        self.four_gait = [[1, -1, -1, 1, -1, -1], [-1, 1, -1, -1, 1, -1], [-1, -1, 1, -1, -1, 1]]
        self.five_gait = [[1, -1, -1, -1, -1, -1], [-1, 1, -1, -1, -1, -1], [-1, -1, 1, -1, -1, -1],
                          [-1, -1, -1, 1, -1, -1], [-1, -1, -1, -1, 1, -1], [-1, -1, -1, -1, -1, 1]]
        self.alpha_j = [np.pi / 6, np.pi / 2, 5 * np.pi / 6, -5 * np.pi / 6, -np.pi / 2, -np.pi / 6]

    def ik_calculate_alpha(self, fxyz_cur, alpha_j):
        theta1 = np.arctan2(fxyz_cur[1] - self.params.R * np.sin(alpha_j),
                            fxyz_cur[0] - self.params.R * np.cos(alpha_j)) - alpha_j
        var_a = fxyz_cur[2] + self.params.L4 * np.cos(self.params.beta - np.sign(alpha_j) * self.params.phi)
        var_b = (fxyz_cur[1] - self.params.R * np.sin(alpha_j)) / np.sin(
            alpha_j + theta1) - self.params.L1 - self.params.L4 * np.sin(
            self.params.beta - np.sign(alpha_j) * self.params.phi)
        var_c = self.params.L2
        var_d = -self.params.L3
        theta3 = np.arcsin(
            (np.power(var_a, 2) + np.power(var_b, 2) - np.power(var_c, 2) - np.power(var_d, 2)) / (-2 * var_c * var_d))
        theta2 = np.arctan((-2 * var_c * var_d * np.cos(theta3)) / (
                np.power(var_a, 2) + np.power(var_b, 2) + np.power(var_c, 2) - np.power(var_d, 2))) + np.arctan2(
            var_a, var_b)
        theta4 = self.params.beta - np.sign(alpha_j) * self.params.phi - theta2 - theta3

        return np.vstack((theta1, theta2, theta3, theta4))

    def ik_calculate(self, fxyz_cur):
        res = np.zeros((6, 4, fxyz_cur.shape[2]), dtype=np.float)
        for i in range(6):
            alpha = self.alpha_j[i]
            theta1 = np.arctan2(fxyz_cur[i][1] - self.params.R * np.sin(alpha),
                                fxyz_cur[i][0] - self.params.R * np.cos(alpha)) - alpha
            var_a = fxyz_cur[i][2] + self.params.L4 * np.cos(self.params.beta - np.sign(alpha) * self.params.phi)
            var_b = (fxyz_cur[i][1] - self.params.R * np.sin(alpha)) / np.sin(
                alpha + theta1) - self.params.L1 - self.params.L4 * np.sin(
                self.params.beta - np.sign(alpha) * self.params.phi)
            var_c = self.params.L2
            var_d = -self.params.L3
            theta3 = np.arcsin((np.power(var_a, 2) + np.power(var_b, 2) - np.power(var_c, 2) - np.power(var_d, 2)) / (
                        -2 * var_c * var_d))
            theta2 = np.arctan((-2 * var_c * var_d * np.cos(theta3)) / (
                        np.power(var_a, 2) + np.power(var_b, 2) + np.power(var_c, 2) - np.power(var_d,
                                                                                                2))) + np.arctan2(var_a,
                                                                                                                  var_b)
            theta4 = self.params.beta - np.sign(alpha) * self.params.phi - theta2 - theta3
            res[i] = np.vstack((theta1, theta2, theta3, theta4))
        return res

    def generate_traj(self, gait_type, Sxyz, T_m=2, period_num=1):
        num = np.int(T_m / 0.01)
        gait_loc = self.three_gait
        if gait_type == 4:
            gait_loc = self.four_gait
        elif gait_type == 5:
            gait_loc = self.five_gait

        loc_period = np.shape(gait_loc)[0]
        period = 0
        # 伪占空比向量
        inv_duty = np.zeros((1, 6), dtype=np.float)
        duty_temp = np.zeros((1, 6), dtype=np.float)

        real_period = loc_period * period_num
        fxyz = np.zeros((6, 3, real_period * (num + 1)), dtype=np.float)

        while period < real_period:
            if (period % loc_period) != 0:
                swisur = gait_loc[period % loc_period][:]
            else:
                swisur = gait_loc[0][:]
            col = period * (num + 1)
            for i in range(6):
                alpha = self.alpha_j[i]
                fx_0 = self.params.R * np.cos(alpha) + (self.params.L1 + self.params.L2) * np.cos(alpha)
                fy_0 = self.params.R * np.sin(alpha) + (self.params.L1 + self.params.L2) * np.sin(alpha)
                fz_0 = -self.params.L3 - self.params.L4
                temp_sign = swisur[i]
                temp_duty = (loc_period - 1) / loc_period * (1 + temp_sign) / 2 + 1 / loc_period * (1 - temp_sign) / 2
                temp_t = T_m / num
                for t in range(num + 1):
                    fxyz[i][0][col + t] = fx_0 + Sxyz[0] * inv_duty[0][i] + Sxyz[0] * temp_duty * temp_sign * (
                            t * temp_t / T_m - 1 / (2 * np.pi) * np.sin(2 * np.pi * t * temp_t / T_m))
                    fxyz[i][1][col + t] = fy_0 + Sxyz[1] * inv_duty[0][i] + Sxyz[1] * temp_duty * temp_sign * (
                            t * temp_t / T_m - 1 / (2 * np.pi) * np.sin(2 * np.pi * t * temp_t / T_m))
                    if temp_sign < 0:
                        fxyz[i][2][col + t] = fz_0
                    else:
                        fxyz[i][2][col + t] = fz_0 + Sxyz[2] * (np.sign(T_m / 2 - t * temp_t) * (2 * (
                                t * temp_t / T_m - 1 / (4 * np.pi) * np.sin(4 * np.pi * t * temp_t / T_m)) - 1) + 1)
                duty_temp[0][i] = temp_duty * temp_sign
                # print(duty_temp)

            inv_duty = inv_duty + duty_temp
            # print(inv_duty)
            period += 1

        return fxyz
    
    def cubic_bezier_traj(self, gait_type, Sxyz, T_m=2, period_num=1):
        num = np.int(T_m / 0.01)
        gait_loc = self.three_gait
        if gait_type == 4:
            gait_loc = self.four_gait
        elif gait_type == 5:
            gait_loc = self.five_gait

        loc_period = np.shape(gait_loc)[0]
        period = 0
        # 伪占空比向量
        inv_duty = np.zeros((1, 6), dtype=np.float)
        duty_temp = np.zeros((1, 6), dtype=np.float)

        real_period = loc_period * period_num
        fxyz = np.zeros((6, 3, real_period * (num + 1)), dtype=np.float)

        while period < real_period:
            if (period % loc_period) != 0:
                swisur = gait_loc[period % loc_period][:]
            else:
                swisur = gait_loc[0][:]
            col = period * (num + 1)
            for i in range(6):
                alpha = self.alpha_j[i]
                fx_0 = self.params.R * np.cos(alpha) + (self.params.L1 + self.params.L2) * np.cos(alpha)
                fy_0 = self.params.R * np.sin(alpha) + (self.params.L1 + self.params.L2) * np.sin(alpha)
                fz_0 = -self.params.L3 - self.params.L4
                temp_sign = swisur[i]
                temp_duty = (loc_period - 1) / loc_period * (1 + temp_sign) / 2 + 1 / loc_period * (1 - temp_sign) / 2
                temp_t = T_m / num
                for t in range(num + 1):
                    real_t = t * temp_t / T_m
                    fxyz[i][0][col + t] = fx_0 + Sxyz[0] * inv_duty[0][i] + Sxyz[0] * temp_duty * temp_sign * (
                            np.power(real_t, 3)+3*np.power(real_t, 2)*(1-real_t))
                    fxyz[i][1][col + t] = fy_0 + Sxyz[1] * inv_duty[0][i] + Sxyz[1] * temp_duty * temp_sign * (
                            np.power(real_t, 3)+3*np.power(real_t, 2)*(1-real_t))
                    if temp_sign < 0:
                        fxyz[i][2][col + t] = fz_0
                    else:
                        if real_t<=0.5:
                            temp_real_t = 2*real_t
                            fxyz[i][2][col + t] = fz_0 + Sxyz[2] * (np.power(temp_real_t, 3)+3*np.power(temp_real_t, 2)*(1-temp_real_t))
                        if real_t>0.5:
                            temp_real_t = 2*real_t-1
                            fxyz[i][2][col + t] = fz_0 + Sxyz[2] - Sxyz[2] * (np.power(temp_real_t, 3)+3*np.power(temp_real_t, 2)*(1-temp_real_t)) 
                duty_temp[0][i] = temp_duty * temp_sign
                # print(duty_temp)

            inv_duty = inv_duty + duty_temp
            # print(inv_duty)
            period += 1

        return fxyz

    def sigmoid_traj(self, gait_type, Sxyz, T_m=2, period_num=1):
        num = np.int(T_m / 0.01)
        gait_loc = self.three_gait
        if gait_type == 4:
            gait_loc = self.four_gait
        elif gait_type == 5:
            gait_loc = self.five_gait

        loc_period = np.shape(gait_loc)[0]
        period = 0
        # 伪占空比向量
        inv_duty = np.zeros((1, 6), dtype=np.float)
        duty_temp = np.zeros((1, 6), dtype=np.float)

        real_period = loc_period * period_num
        fxyz = np.zeros((6, 3, real_period * (num + 1)), dtype=np.float)

        while period < real_period:
            if (period % loc_period) != 0:
                swisur = gait_loc[period % loc_period][:]
            else:
                swisur = gait_loc[0][:]
            col = period * (num + 1)
            for i in range(6):
                alpha = self.alpha_j[i]
                fx_0 = self.params.R * np.cos(alpha) + (self.params.L1 + self.params.L2) * np.cos(alpha)
                fy_0 = self.params.R * np.sin(alpha) + (self.params.L1 + self.params.L2) * np.sin(alpha)
                fz_0 = -self.params.L3 - self.params.L4
                temp_sign = swisur[i]
                temp_duty = (loc_period - 1) / loc_period * (1 + temp_sign) / 2 + 1 / loc_period * (1 - temp_sign) / 2
                temp_t = T_m / num
                for t in range(num + 1):
                    gama = 1/(1+np.exp(-30*(t/num-0.5)))
                    fxyz[i][0][col + t] = fx_0 + Sxyz[0] * inv_duty[0][i] + Sxyz[0] * temp_duty * temp_sign * gama
                    fxyz[i][1][col + t] = fy_0 + Sxyz[1] * inv_duty[0][i] + Sxyz[1] * temp_duty * temp_sign * gama
                    if temp_sign < 0:
                        fxyz[i][2][col + t] = fz_0
                    else:
                        fxyz[i][2][col + t] = fz_0 + 0.2*(-0.5*np.cos(2*np.pi*t/num)+0.5)
                duty_temp[0][i] = temp_duty * temp_sign
                # print(duty_temp)

            inv_duty = inv_duty + duty_temp
            # print(inv_duty)
            period += 1

        return fxyz
        
    def  kinematics_cal(self, theta1, theta2, theta3, theta4, alpha):
        fx = self.params.R * np.cos(alpha) + (self.params.L1+self.params.L2*np.cos(theta2)+self.params.L3*np.sin(theta2+theta3)+self.params.L4*np.sin(theta2+theta3+theta4))*np.cos(alpha+theta1)
        fy = self.params.R * np.sin(alpha) + (self.params.L1+self.params.L2*np.cos(theta2)+self.params.L3*np.sin(theta2+theta3)+self.params.L4*np.sin(theta2+theta3+theta4))*np.sin(alpha+theta1)
        fz = self.params.L2*np.sin(theta2) - self.params.L3*np.cos(theta2+theta3) - self.params.L4*np.cos(theta2+theta3+theta4)
        return [fx, fy, fz]

if __name__ == '__main__':
    a = IK()
    x = a.sigmoid_traj(3, [0.2, 0, 0.2], T_m=2, period_num=1)
    # x = a.cubic_bezier_traj(3, [0.15, 0, 0.2], T_m=2, period_num=1)
    print(x.shape)
    # b = x[0][:][:]
    # y = a.ik_calculate(x)
    # print(y.shape)
    ax = plt.axes(projection='3d')
    ax.plot3D(x[0][0][:], x[0][1][:], x[0][2][:])
    plt.show()

    # fig = plt.figure()
    # ax = fig.add_subplot(1, 1, 1)
    # ax.plot(y[0][2][:])
    # plt.show()
    # b=[0.3551, 0.2050, -0.310]
    # b = np.random.rand(3, 100)
    # c = np.pi/6
    # d = a.ik_calculate(b, c)
    # print(d.shape)
