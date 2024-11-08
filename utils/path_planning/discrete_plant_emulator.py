import csv
import multiprocessing
import struct
import time
from multiprocessing import shared_memory
import numpy as np
from matplotlib import pyplot as plt
from path_planning.pidf_controller import PidfControl


class DiscretePlant:
    def __init__(self, ts, k=1, a=1):
        # Emulates a discrete plant of the form: k/(s^2+as).
        alpha = 1/(2*a*ts + 4)
        # Calculating u and y coefficients:
        self.u0 = alpha*k*ts**2
        self.u1 = 2*alpha*k*ts**2
        self.u2 = alpha*k*ts**2
        self.y1 = alpha*8
        self.y2 = alpha*(2*a*ts-4)

        # Saving previous u and y values:
        self.u_k1 = 0  # Meaning U_k-1
        self.u_k2 = 0
        self.y_k1 = 0
        self.y_k2 = 0

    def iterate_step(self, u):
        # Calculate the next value:
        y = self.u0 * u + self.u1 * self.u_k1 + self.u2 * self.u_k2 + self.y1 * self.y_k1 + self.y2 * self.y_k2
        # Push values of u and y one step backwards:
        self.u_k2 = self.u_k1
        self.u_k1 = u
        self.y_k2 = self.y_k1
        self.y_k1 = y
        return y

    def async_steering(self, sample_time, controller):
        # Cannot use SteeringProcManager() due to a circular import...
        shmem_active = shared_memory.SharedMemory(name='active_state', create=False)
        shmem_setpoint = shared_memory.SharedMemory(name='input_value', create=False)
        shmem_output = shared_memory.SharedMemory(name='output_value', create=False)

        output = struct.unpack('d', shmem_output.buf[:8])[0]
        is_active = struct.unpack('?', shmem_active.buf[:1])[0]
        last_iteration = time.perf_counter()

        while is_active:
            iteration_time = time.perf_counter() - last_iteration
            if iteration_time > sample_time:
                # Poll the setpoint input from shared memory:
                setpoint = struct.unpack('d', shmem_setpoint.buf[:8])[0]
                # Pass input and output signals through controller:
                compensated_signal = controller.position_control(setpoint, output)
                # Calculate the output of the plant's next iteration:
                output = self.iterate_step(compensated_signal)
                # Write result into shared memory:
                shmem_output.buf[:8] = struct.pack('d', output)
                # Check whether loop should go on:
                is_active = struct.unpack('?', shmem_active.buf[:1])[0]
                # Update time count for next iteration:
                last_iteration = time.perf_counter()

        # Close the process' use of the shared memory once the loop is complete:
        shmem_active.close()
        shmem_setpoint.close()
        shmem_output.close()


def run_subprocess():
    duration = 2.0
    inputs = np.array([], dtype=float)
    outputs = np.array([], dtype=float)
    # steer_emulator = DiscretePlant(0.01)
    # steer_controller = PidfControl(0.01)
    # steer_controller.set_pidf(900.0, 0.0, 42.0, 0.0)
    # steer_controller.set_extrema(0.01, 1.0)
    # steer_controller.alpha = 0.1
    dt = 0.001
    steer_emulator = DiscretePlant(dt, 10, 4)
    steer_controller = PidfControl(dt)
    steer_controller.set_pidf(1000.0, 0.0, 15, 0.0)
    steer_controller.set_extrema(0.01, 1.0)
    steer_controller.alpha = 0.01

    shmem_active = shared_memory.SharedMemory(name='active_state', create=True, size=1)
    shmem_setpoint = shared_memory.SharedMemory(name='input_value', create=True, size=8)
    shmem_output = shared_memory.SharedMemory(name='output_value', create=True, size=8)

    is_active = True
    setpoint = 0.0
    output = 0.0

    shmem_active.buf[:1] = struct.pack('?', is_active)
    shmem_setpoint.buf[:8] = struct.pack('d', setpoint)
    shmem_output.buf[:8] = struct.pack('d', output)

    steering_thread = multiprocessing.Process(target=steer_emulator.async_steering,
                                              args=(dt, steer_controller),
                                              daemon=True)
    steering_thread.start()
    time.sleep(2.0)  # New process takes a lot of time to "jumpstart"

    try:
        start_time = time.perf_counter()
        last_iteration = time.perf_counter()
        run_time = 0.0
        idx = 0
        setpoint = 10.0
        while run_time < duration:
            run_time = time.perf_counter() - start_time
            iteration_time = time.perf_counter() - last_iteration
            if iteration_time > dt:
                if run_time > 0.5:
                    setpoint = -20.0
                if run_time > 1.0:
                    setpoint = 30.0
                if run_time > 1.5:
                    setpoint = -40.0

                shmem_setpoint.buf[:8] = struct.pack('d', setpoint)
                output = struct.unpack('d', shmem_output.buf[:8])[0]

                inputs = np.append(inputs, setpoint)
                outputs = np.append(outputs, output)
                last_iteration = time.perf_counter()
                idx += 1

        # End parallel process loop
        is_active = False
        shmem_active.buf[:1] = struct.pack('?', is_active)

        # Plot the result
        timeline = np.linspace(0, duration, idx)
        fig, ax = plt.subplots(1, 1)
        ax.plot(timeline, inputs, '-r')
        ax.plot(timeline, outputs, '-b')
        ax.grid(True)
        fig.show()
        plt.waitforbuttonpress()

    finally:
        shmem_active.buf[:1] = struct.pack('?', False)
        shmem_setpoint.unlink()
        shmem_output.unlink()
        shmem_active.unlink()
        time.sleep(1.0)


def run_offline():
    duration = 2.0
    dt = 0.001
    inputs = np.array([], dtype=float)
    outputs = np.array([], dtype=float)
    steer_emulator = DiscretePlant(dt, 10, 4)
    # steer_emulator = DiscretePlant(dt, 1, 1)
    steer_controller = PidfControl(dt)
    # steer_controller.set_pidf(900.0, 0.0, 42.0, 0.0)
    # steer_controller.set_pidf(600.0, 0.0, 35.0, 0.0)
    steer_controller.set_pidf(1000.0, 0.0, 15.0, 0.0)
    steer_controller.set_extrema(0.01, 1.0)
    steer_controller.alpha = 0.01

    setpoint = 10.0
    output = 0.0
    idx = 0

    while idx * dt < duration:
        if idx * dt > 0.5:
            setpoint = -20.0
        if idx * dt > 1.0:
            setpoint = 30.0
        if idx * dt > 1.5:
            setpoint = -40.0

        compensated_signal = steer_controller.position_control(setpoint, output)
        output = steer_emulator.iterate_step(compensated_signal)

        idx += 1
        inputs = np.append(inputs, setpoint)
        outputs = np.append(outputs, output)

    # Plot the result
    save_data = np.append(inputs.reshape((inputs.size, 1)), outputs.reshape((outputs.size, 1)), axis=1)
    with open('compensated_discrete.csv', 'w', newline='') as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(['inputs', 'outputs'])
        writer.writerows(save_data)
    print('saved csv data')
    timeline = np.linspace(0, duration, idx)
    fig, ax = plt.subplots(1, 1)
    ax.plot(timeline, inputs, '-r')
    ax.plot(timeline, outputs, '-b')
    ax.grid(True)
    fig.show()
    plt.waitforbuttonpress()


if __name__ == '__main__':
    run_subprocess()
    # run_offline()
