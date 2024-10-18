#!/usr/bin/env python
# -*- coding: utf-8 -*-

import matplotlib.pyplot as plt
import numpy as np
import rospy
from std_msgs.msg import Int16MultiArray


class AudioPlotter:
    def __init__(self):
        rospy.init_node("audio_plotter", anonymous=True)
        self.buffer_size = 1024  # 受信するサンプル数に合わせて調整
        self.audio_data = np.zeros(self.buffer_size)

        # プロットの設定
        self.fig, self.ax = plt.subplots()
        (self.line,) = self.ax.plot(self.audio_data)
        self.ax.set_ylim([-32768, 32767])  # 16ビットPCMの範囲
        self.ax.set_title("Real-time Audio Data")
        self.ax.set_xlabel("Sample Index")
        self.ax.set_ylabel("Amplitude")

        # トピックの購読
        rospy.Subscriber("/audio", Int16MultiArray, self.callback)

        # プロットを非ブロッキングモードに設定
        plt.ion()
        plt.show()

    def callback(self, data):
        rospy.loginfo("hoge")
        # 受信したデータを NumPy 配列に変換
        self.audio_data = np.array(data.data)
        # プロットを更新
        self.line.set_ydata(self.audio_data)
        self.line.set_xdata(np.arange(len(self.audio_data)))
        self.ax.relim()
        self.ax.autoscale_view()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()


if __name__ == "__main__":
    try:
        plotter = AudioPlotter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
