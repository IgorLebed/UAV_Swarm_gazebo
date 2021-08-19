                if (exit_p_num == 1):
                    rospy.loginfo("Hold mode\nExit from program")
                    self.set_mode("OFFBOARD", 5)
                    #work = False
                    check_1 =True
                    while (check_1 == True):
                        check_2 = True
                        while (check_2 == True):#(self.goal_pose_x == None and self.goal_pose_y == None):
                            try:
                                check_3 = True
                                while (check_3 == True):
                                    if (self.scout0_check.pose.position.x != 1):
                                        while (self.scout0_check.pose.position.x != 1):
                                            self.set_mode("OFFBOARD", 5)
                                            scout_pose_x =  self.local_scout0_position.pose.position.x
                                            scout_pose_y =  self.local_scout0_position.pose.position.y
                                            scout_pose_z =  self.local_scout0_position.pose.position.z # test
                                            self.reach_position(int(scout_pose_x) + 2, int(scout_pose_y), int(scout_pose_z) - 2, 50) # X, Y, Z
                                            rospy.loginfo("local pos x: %s and pos y: %s ", scout_pose_x, scout_pose_y)
                                            time.sleep(0.3)
                                        if self.scout0_check.pose.position.x == 1:
                                            check_3 = False
                                    else:
                                        while (self.scout0_check.pose.position.x == 1):
                                            self.set_mode("OFFBOARD", 5)
                                            scout_pose_x =  self.goal_pose_x
                                            scout_pose_y =  self.goal_pose_y
                                            scout_pose_z =  13 #test
                                            self.reach_position(int(scout_pose_x) + 2, int(scout_pose_y), int(scout_pose_z) - 2, 50) # X, Y, Z 
                                            time.sleep(0.3)
                                        if (self.scout0_check.pose.position.x != 1):
                                            check_2 = False   
                            except rospy.ROSInterruptException:
                                rospy.loginfo("This is not num!")
                                check_2 = True
                        check_1 = False