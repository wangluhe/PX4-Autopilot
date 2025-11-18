#!/usr/bin/env python3

import asyncio
from mavsdk import System
from mavsdk.offboard import VelocityBodyYawspeed

async def run():
	# 连接到无人机
	drone = System()
	await drone.connect(system_address="udp://:14540")

	print("等待无人机连接...")
	async for state in drone.core.connection_state():
		if state.is_connected:
			print("无人机已连接!")
		break

	print("等待GPS定位...")
	async for health in drone.telemetry.health():
		if health.is_global_position_ok and health.is_home_position_ok:
			print("GPS定位就绪")
		break

	# 解锁无人机
	print("解锁中...")
	await drone.action.arm()

	# 设置Offboard模式
	print("设置Offboard模式...")
	await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
	await drone.offboard.start()

	# 发布X方向速度指令
	print("向前运动 - X方向1m/s")
	await drone.offboard.set_velocity_body(VelocityBodyYawspeed(1.0, 0.0, 0.0, 0.0))

	# 持续5秒
	await asyncio.sleep(5)

	# async for battery in drone.telemetry.battery():
	# 	print(f"电池: {battery.remaining_percent*100:.1f}%")

	# 获取速度信息
	async for velocity in drone.telemetry.velocity_ned():
		print(f"速度: 北={velocity.north_m_s:.1f}, 东={velocity.east_m_s:.1f}, 下={velocity.down_m_s:.1f} m/s")


	# # 停止运动
	# print("停止运动")
	# await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))

	# # 返回Land模式
	# print("返回Land模式")
	# await drone.offboard.stop()
	# await drone.action.land()

	# # 锁定
	# await asyncio.sleep(5)
	# await drone.action.disarm()

if __name__ == "__main__":
	asyncio.run(run())
