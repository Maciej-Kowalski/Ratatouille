def EKF(input_queue, output_queue, stop_event):
    flag = 1
    gyr_prev = []
    acc_prev = []
    gyr_cur = []
    acc_cur = []

    while not stop_event.is_set():
        task = input_queue.get()
        #Your code
        if task is not None:
            if flag == 1:
                acc, gyro = task
                acc_array = np.array([acc],dtype = float)
                gyro_array = np.array([gyro],dtype = float)
                acc_prev = acc_array
                gyr_prev = gyro_array
                flag = 0

            acc, gyro = task
            acc_array = np.array([acc],dtype = float)
            gyro_array = np.array([gyro],dtype = float)

            # Pre-filter
            acc_cur = (acc_alpha * acc_prev) + ((1 - acc_alpha) * (acc_array))
            gyr_cur = (gyr_alpha * gyr_prev) + ((1 - gyr_alpha) * (gyro_array))

            acc_prev = acc_cur
            gyr_prev = gyr_cur

            ekf = EKF(gyr_cur, acc_cur,frequency=19.0,frame='ENU')

            #Your code END

            inputqueue.task_done()
            outputqueue.put(your_data)

        else:
            print("IMUData slow")