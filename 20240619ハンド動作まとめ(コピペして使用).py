

#内爪開
now_pos_dxl = dxl.read(Motor_ID, dxl.Address.PresentPosition)
dxl.PosCnt_Vbase(Motor_ID,now_pos_dxl - inner_finger_dis,experiment_motor_speed)
t_p_start = time.time()
while True:
    now_velocity = dxl.read(Motor_ID, dxl.Address.PresentVelocity)
    program_time = time.time() - t_p_start

    if keyboard.is_pressed("q"):  # qが押されたら終了
        break

    if program_time >= 0.5 and now_velocity == 0:
        now_pos_dxl = dxl.read(Motor_ID, dxl.Address.PresentPosition)
        dxl.Change_OperatingMode(Motor_ID, dxl.operating_mode.position_control)
        dxl.write(Motor_ID, dxl.Address.GoalPosition, now_pos_dxl)
        break

#内爪閉
now_pos_dxl = dxl.read(Motor_ID, dxl.Address.PresentPosition)
dxl.PosCnt_Vbase(Motor_ID,now_pos_dxl + inner_finger_dis,experiment_motor_speed)
t_p_start = time.time()
while True:
    now_velocity = dxl.read(Motor_ID, dxl.Address.PresentVelocity)
    program_time = time.time() - t_p_start

    if keyboard.is_pressed("q"):  # qが押されたら終了
        break

    if program_time >= 0.5 and now_velocity == 0:
        now_pos_dxl = dxl.read(Motor_ID, dxl.Address.PresentPosition)
        dxl.Change_OperatingMode(Motor_ID, dxl.operating_mode.position_control)
        dxl.write(Motor_ID, dxl.Address.GoalPosition, now_pos_dxl)
        break

#外爪開(内爪閉状態から)
now_pos_dxl = dxl.read(Motor_ID, dxl.Address.PresentPosition)
dxl.PosCnt_Vbase(Motor_ID,now_pos_dxl + outer_finger_dis,experiment_motor_speed)
t_p_start = time.time()
while True:
    now_velocity = dxl.read(Motor_ID, dxl.Address.PresentVelocity)
    program_time = time.time() - t_p_start

    if keyboard.is_pressed("q"):  # qが押されたら終了
        break

    if program_time >= 0.5 and now_velocity == 0:
        now_pos_dxl = dxl.read(Motor_ID, dxl.Address.PresentPosition)
        dxl.Change_OperatingMode(Motor_ID, dxl.operating_mode.position_control)
        dxl.write(Motor_ID, dxl.Address.GoalPosition, now_pos_dxl)
        break