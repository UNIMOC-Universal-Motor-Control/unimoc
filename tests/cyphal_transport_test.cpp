#include <gtest/gtest.h>
/**
 * @file cyphal_transport_test.cpp
 * @brief GoogleTest coverage for the injected Cyphal transport boundary.
 */
#include <array>
#include <cstddef>
#include <cstring>
#include <span>
#include "cyphal/node.hpp"

namespace unimoc::cyphal::test {
namespace {

class FakeTransport final : public Transport {
 public:
  bool Send(const Transfer& transfer) noexcept override {
    sent_transfer = transfer;
    return send_result;
  }

  ReceiveResult Receive(std::span<std::byte> payload_buffer, Transfer& transfer) noexcept override {
    if (receive_result != ReceiveResult::kReceived) {
      return receive_result;
    }
    if (payload.size() > payload_buffer.size()) {
      return ReceiveResult::kError;
    }
    std::memcpy(payload_buffer.data(), payload.data(), payload.size());
    transfer = received_transfer;
    transfer.payload = payload_buffer.first(payload.size());
    return ReceiveResult::kReceived;
  }

  bool send_result{true};
  ReceiveResult receive_result{ReceiveResult::kNoTransfer};
  Transfer sent_transfer{};
  Transfer received_transfer{};
  std::span<const std::byte> payload;
};

TEST(CyphalNodeTest, SendsThroughInjectedTransport) {
  FakeTransport fake;
  CyphalNode node{fake};
  const std::array<std::byte, 2U> kPayload{std::byte{0x12}, std::byte{0x34}};
  const Transfer kTransfer{
      .port_id = 42U,
      .source_node_id = 10U,
      .destination_node_id = 20U,
      .transfer_id = 3U,
      .kind = TransferKind::kServiceRequest,
      .payload = kPayload,
  };

  ASSERT_TRUE(node.Send(kTransfer));
  EXPECT_EQ(fake.sent_transfer.port_id, 42U);
  EXPECT_EQ(fake.sent_transfer.source_node_id, 10U);
  EXPECT_EQ(fake.sent_transfer.destination_node_id, 20U);
  EXPECT_EQ(fake.sent_transfer.kind, TransferKind::kServiceRequest);
  ASSERT_EQ(fake.sent_transfer.payload.size(), kPayload.size());
  EXPECT_EQ(std::memcmp(fake.sent_transfer.payload.data(), kPayload.data(), kPayload.size()), 0);
}

TEST(CyphalNodeTest, PollsIntoCallerOwnedBuffer) {
  FakeTransport fake;
  CyphalNode node{fake};
  const std::array<std::byte, 3U> kInput{std::byte{0x01}, std::byte{0x02}, std::byte{0x03}};
  fake.receive_result = ReceiveResult::kReceived;
  fake.received_transfer = Transfer{
      .port_id = 100U,
      .source_node_id = 7U,
      .destination_node_id = 255U,
      .transfer_id = 9U,
      .kind = TransferKind::kMessage,
      .payload = {},
  };
  fake.payload = kInput;
  std::array<std::byte, 8U> k_buffer{};
  Transfer received{};

  EXPECT_EQ(node.Poll(k_buffer, received), ReceiveResult::kReceived);
  EXPECT_EQ(received.port_id, 100U);
  EXPECT_EQ(received.source_node_id, 7U);
  ASSERT_EQ(received.payload.size(), kInput.size());
  EXPECT_EQ(std::memcmp(received.payload.data(), kInput.data(), kInput.size()), 0);
  EXPECT_EQ(std::memcmp(k_buffer.data(), kInput.data(), kInput.size()), 0);
}

}  // namespace
}  // namespace unimoc::cyphal::test