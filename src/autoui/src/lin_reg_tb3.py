#! /usr/bin/env python

import roslib
import rospy
import pandas as pd
import numpy as np
from sklearn.linear_model import LinearRegression, LogisticRegression

hl_xy = [[1.88313, 0.00913], [3.8226, -0.0063], [7.1109, 0.06975], [8.28955, 0.07335]
        , [7.20103, 0.04687], [5.50628, 0.07548], [7.25371, 0.09304], [7.68623, -0.15549]
        , [3.5584, 0.04884], [3.16858, 0.10277], [5.98738, 0.074],[5.80622, 0.05189]]
tb_xy = [[1.91905, -0.14872], [3.8975, -0.008975], [7.20257, 0.155683], [8.38084, 0.20240]
        , [7.34152, 0.20863], [5.75029, 0.08510], [7.53829, 0.104706], [7.9324, -0.07891]
        , [3.7215, 0.25277], [3.3769, 0.304802], [6.1843, 0.24772], [6.21974, 0.1043]]

reg = LinearRegression().fit(hl_xy, tb_xy)
print(reg.score(hl_xy, tb_xy))
print(reg.coef_)
print(reg.intercept_)

tb_xy_copy = tb_xy.copy()
hl_xy_copy = hl_xy.copy()

# reg_tb_points = pd.DataFrame(tb_xy_copy,columns=['TbX','TbY'])
# reg_hl_points = pd.DataFrame(hl_xy_copy,columns=['HlX','HlY'])
# reg_points_df = reg_hl_points.join(reg_tb_points)
# print(reg_points_df)
# # print(reg_dobot_points)
# # print(reg_screen_points)

# reg_points_df.to_csv('RegPointSaved.csv')

# saved_reg_points = pd.read_csv('RegPointSaved.csv', index_col = 0)
# saved_reg_points[['HlX','HlY']]
# reg = LinearRegression().fit(saved_reg_points[['HlX','HlY']], saved_reg_points[['TbX','TbY']])
# print(reg.score(saved_reg_points[['HlX','HlY']],saved_reg_points[['TbX','TbY']]))
hl_x = float(input("Enter X (hololens)"))
hl_y = float(input("Enter Y (hololens)"))
pred_tb_xy = reg.predict([[hl_x,hl_y]])
print('predicted Turtlebot X,Y = ', pred_tb_xy)


