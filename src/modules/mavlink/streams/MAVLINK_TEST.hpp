// 在MAVLINK_TEST.hpp中填入下面内容，
// 主要是订阅test_mavlink_tx消息然后将消息内容打包为MAVLINK数据帧并发送
#ifndef MAVLINK_TEST_HPP
#define MAVLINK_TEST_HPP

#include <uORB/topics/test_mavlink_tx.h>
class MavlinkStreamMavlinktest : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamMavlinktest(mavlink); }

	static constexpr const char *get_name_static() { return "TEST_MAVLINK"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_TEST_MAVLINK; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return test_mavlink_tx_sub.advertised() ? MAVLINK_MSG_ID_TEST_MAVLINK_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamMavlinktest(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription test_mavlink_tx_sub{ORB_ID(test_mavlink_tx)};

	bool send() override
	{
		test_mavlink_tx_s test_mavlink_tx;

		if (test_mavlink_tx_sub.update(&test_mavlink_tx)) {
			mavlink_test_mavlink_t msg{};
msg.test1=test_mavlink_tx.test1;
msg.test2=test_mavlink_tx.test2;
msg.test3=test_mavlink_tx.test3;
			mavlink_msg_test_mavlink_send_struct(_mavlink->get_channel(), &msg);
			return true;
		}

		return false;
	}
};

#endif
