import threading
import time

'''
t1 = 0

# 主程序
def main_program():
    global t1, t2
    event2.set()
    while True:
        if t1 == 1:
            print("主线程开始仿真结束，其他线程等待")
            time.sleep(5)
            event3.set()
            print("主线程仿真结束，其他线程开始执行")

            print("主线程开始数据处理，其他线程等待")
            time.sleep(1)
            event.set()
            print("主线程数据处理结束，其他线程开始执行")

            print("主线程开始开启仿真，其他线程等待")
            time.sleep(5)  
            print("主线程开启仿真结束，其他线程开始执行")
            event2.set()


# 线程1
def thread1():
    global t1
    while True:
        t1 = 0
        print("线程1开始等待仿真开启,等待中....")
        event2.wait()
        event2.clear()
        print("线程1等待仿真开启结束,继续执行...")
        time.sleep(5)

        t1 = 1
        print("线程1开始等待仿真结束,等待中....")
        event3.wait()
        event3.clear()
        print("线程1等待仿真结束,继续执行...")

        print("线程1开始等待数据处理,等待中....")
        event.wait() 
        event.clear()
        print("线程1等待数据处理结束,继续执行...")
        time.sleep(1)


if __name__ == "__main__":
    # 创建一个事件对象
    event = threading.Event()
    event2 = threading.Event()
    event3 = threading.Event()

    # 创建线程
    t1 = threading.Thread(target=thread1)

    # 启动线程
    t1.start()

    # 执行主程序
    main_program()
'''
t1 = 0
t2 = 0

t1_cnt = 0
t2_cnt = 0
t3_cnt = 0
# 主程序
def main_program():
    global t1, t2, t3_cnt, t1_cnt, t2_cnt
    event2.set()
    while True:
        if t1 == 1 and t2 == 1:
            print("主线程开始仿真结束，其他线程等待")
            time.sleep(5)
            event3.set()
            print("主线程仿真结束，其他线程开始执行")

            print("主线程开始数据处理，其他线程等待")
            time.sleep(1)
            event.set()
            print("主线程数据处理结束，其他线程开始执行")

            print("主线程开始开启仿真，其他线程等待")
            time.sleep(5)  
            print("主线程开启仿真结束，其他线程开始执行")
            event2.set()
        
        if t1_cnt == 2:
            event.clear()
            t1_cnt = 0
        
        if t2_cnt == 2:
            event2.clear()
            t2_cnt = 0
        
        if t3_cnt == 2:
            event3.clear()
            t3_cnt = 0


# 线程1
def thread1():
    global t1, t3_cnt, t1_cnt, t2_cnt
    while True:
        t1 = 0
        print("线程1开始等待仿真开启,等待中....")
        event2.wait()
        t2_cnt += 1
        print("线程1等待仿真开启结束,继续执行...")
        time.sleep(5)

        t1 = 1
        print("线程1开始等待仿真结束,等待中....")
        event3.wait()
        t3_cnt += 1
        print("线程1等待仿真结束,继续执行...")

        print("线程1开始等待数据处理,等待中....")
        event.wait() 
        t1_cnt += 1
        print("线程1等待数据处理结束,继续执行...")
        time.sleep(1)

# 线程1
def thread2():
    global t2, t3_cnt, t1_cnt, t2_cnt
    while True:
        t2 = 0
        print("线程2开始等待仿真开启,等待中....")
        event2.wait()
        t2_cnt += 1
        print("线程2等待仿真开启结束,继续执行...")
        time.sleep(5)

        t2 = 1
        print("线程2开始等待仿真结束,等待中....")
        event3.wait()
        t3_cnt += 1
        print("线程2等待仿真结束,继续执行...")

        print("线程2开始等待数据处理,等待中....")
        event.wait() 
        t1_cnt += 1
        print("线程2等待数据处理结束,继续执行...")
        time.sleep(1)


if __name__ == "__main__":
    # 创建一个事件对象
    event = threading.Event()
    event2 = threading.Event()
    event3 = threading.Event()

    # 创建线程
    t1 = threading.Thread(target=thread1)
    t2 = threading.Thread(target=thread2)

    # 启动线程
    t1.start()
    t2.start()

    # 执行主程序
    main_program()

