# ros2_subscription_examples

Example programs for demonstrating data read from subscription of ROS 2.

## packages

These example programs consist of four pakages.

- [simple_examples](./simple_examples)
- [intra_process_talker_listener](./intra_process_talker_listener)
- [intra_process_examples](./intra_process_examples)
- [waitset_examples](./waitset_examples)

These are based on taker-listener in [GitHub - ros2/demos at humble](https://github.com/ros2/demos/tree/humble). If you know the talker-listener well, you can also understand these sample codes. Only `intra_process_examples` is based on [demos/intra_process_demo/src/two_node_pipeline/two_node_pipeline.cpp at humble · ros2/demos](https://github.com/ros2/demos/blob/humble/intra_process_demo/src/two_node_pipeline/two_node_pipeline.cpp) exceptionally.

## how-to build

```console
mkdir ros2_subscription_test
cd ros2_subscription_test
git clone git@github.com:takam5f2/ros2_subscription_examples.git
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## how-to run

### `simple_examples`

You need two terminals one of which is for talker and another for listener.

#### run talker

Execute command below to run talker node on a terminal.

```console
ros2 run simple_examples [talker node]
```

The talker supports some additional options unlike the talker of [GitHub - ros2/demos at humble](https://github.com/ros2/demos/tree/humble). For example, you can change frequency of messages transmission by `update_frequency` option as below.

```console
ros2 run simple_examples talker --ros-args -p update_frequency:=2.0
```

If you want to use multiple options, you can add subsequent options as `-p option_args:=<value>`.

#### talker nodes list

Below is the list of talker nodes included in simple_examples package.

- `talker`
  - behavior
    - it outputs a `/chatter` topic message per specified frequency
  - option
    - `update_frequency` (float)
      - specify output frequency of messages
      - default is 1.0
    - `use_transient_local` (boolean)
      - enable Transient Local of Publisher
      - default is false
- `serialized_message_talker`
  - behavior
    - output `/chatter` of SerializedMessage type per one second
  - no option

#### run listener

Execute command below to run listener node on another terminal.

```console
ros2 run simple_examples [listener node]
```

The listener of [ros2_subscription_examples/simple_examples at main · takam5f2/ros2_subscription_examples](https://github.com/takam5f2/ros2_subscription_examples/tree/main/simple_examples) is driven by timer unlike the listener of [GitHub - ros2/demos at humble](https://github.com/ros2/demos/tree/humble) which is driven by subscription of `/chatter` topic message.

#### listener nodes list

Below is the list of listener nodes included in simple_examples package.

- `timer_listener`
  - behavior
    - It has a cyclic timer to execute a callback function in which a message is taken from a subscription
    - It prints out string data, included in received `/chatter`, to the terminal
  - option
    - `update_frequency` (float)
      - specify timer frequency
      - default is 1.0
    - `use_transient_local` (boolean)
      - enable Transient Local of Subscription
      - default is false
    - `printout_message_info` (boolean)
      - enable to print MessageInfo
      - default is false
- `timer_batch_listener`
  - behavior
    - It has a cyclic timer to execute a callback function in which multiple messages are obtained from a subscription until the subscription queue becomes empty
    - It prints out string data, included in received `/chatter`, to the terminal
  - option
    - `update_frequency` (float)
      - specify timer frequency
      - default is 0.2
    - `use_transient_local` (boolean)
      - enable Transient Local of Subscription
      - default is false
    - `queue_size` (integer)
      - specify Subscription Queue size which stores `/chatter` message
- `timer_listener_using_callback`
  - behavior
    - It has a cyclic timer to execute a callback function in which a message is taken from the subscription using `take_type_erased()`
    - `handle_message()` is called to execute a callback function which is registered to the subscription during execution of the timer-based callback function
  - option
    - `update_frequency` (float)
      - specify timer frequency
      - default is 1.0
    - `use_transient_local` (boolean)
      - enable Transient Local of Subscription
      - default is false
- `timer_listener_using_normal_function`
  - behavior
    - It has a cyclic timer to execute a callback function in which a message is obtained from Subscription
    - Another subroutine is called from the callback function to print out string data included in a received message
    - this listener is similar to `timer_listener_using_callback`, but it does not use `handle_message()`
  - option
    - `update_frequency` (float)
      - specify timer frequency
      - default is 1.0
    - `use_transient_local` (boolean)
      - enable Transient Local of Subscription
      - default is false
- `timer_serialized_message_listener`
  - behavior
    - It has a cyclic timer to execute a callback function in which a message is obtained from Subscription
    - the data type of the message is SerializedMessage
    - this listener should run together with `serialized_message_talker`
  - option
    - `update_frequency` (float)
      - specify timer frequency
      - default is 0.2
    - `use_transient_local` (boolean)
      - enable Transient Local of Subscription
      - default is false
    - `queue_size` (integer)
      - specify Subscription Queue size which stores `/chatter` message

### `intra_process_talker_listener`

There are a few sample programs in `intra_process_talker_listener` in which a topic message is obtained by `take_data()` method and then processed by a callback function through `execute()` method. Intra-process communication requires a pair of publisher and subscription is located on a same Linux process. Then a pair of listener and talker must be launched as composable nodes. The package provides two sample launch files to start listener and talker on a composable nodes.

#### run by launcher

Execute command below to run `talker_listener_intra_process.launch.py`.

```console
ros2 launch intra_process_talker_listener talker_listener_intra_process.launch.py
```

The launch file supports some options. Below is an example to use `use_intra_process_comms` option.

```console
ros2 launch intra_process_talker_listener talker_listener_intra_process.launch.py use_intra_process_comms:=false
```

If `use_intra_process_comms` is set to `false`, communication between nodes is performed through DDS. If it is set to `true`, communication is performed through intra-process communication.

#### launch files list

Below is the list of launch files included in intra_process_talker_listener package.

- talker_listener_intra_process.launch.py
  - behavior
    - It launches `talker_intra_process` and `timer_listener_intra_process`
    - `talker_intra_process` transmits `/chatter` message and `timer_listener_intra_process` receives it
    - if `use_intra_process_comms` is set to `true`, intra-process communication is performed between the two nodes
    - if `use_intra_process_comms` is set to `false`, a communication through DDS is performed between the two nodes
  - option
    - `use_intra_process_comms` (boolean)
      - enable intra-process communication
      - default is true
    - `talker_update_frequency` (float)
      - specify frequency of publishing `/chatter` message by `talker_intra_process`
      - default is 1.0
    - `listener_update_frequency` (float)
      - specify timer frequency of `timer_listener_intra_process`
      - default is 1.0
    - `use_transient_local` (boolean)
      - enable Transient Local of Publisher and Subscription
      - default is false
- talker_batch_listener_intra_process.launch.py
  - behavior
    - It launches `talker_intra_process` and `timer_batch_listener_intra_process`
    - `talker_intra_process` transmits `/chatter` message and `timer_batch_listener_intra_process` receives it
    - `timer_batch_listener_intra_process` executes a callback function per five seconds in which topic messages are obtained from Subscription Queue until the queue becomes empty
  - option
    - `use_intra_process_comms` (boolean)
      - enable intra-process communication
      - default is true

### `intra_process_example`

As well as using composable nodes, intra-process communication also can be used between nodes on MultiThreadedExecutor to which they are registered, which you can see in `intra_process_example`.
`/consumer` and `/producer` nodes run in `intra_process_example` and `/producer` publishes a topic message and `/consumer` obtains it.

#### run

Execute `two_node_pipeline_timer` with the following command.

```console
 ros2 run intra_process_examples two_node_pipeline_timer
```

It has no particular option. ID of the thread which executes a timer callback function and that of the thread which executes a subscription callback function are displayed in this program. The IDs should match.

### `waitset_examples`

You need two terminals; one is for talker and the other for listener.

#### run talker

Launch a terminal and execute talker node with the following command.

```console
ros2 run waitset_examples talker_triple
```

#### talker node list

Only `talker_triple` is provided as talker node in the `waitset_examples` package.

- `talker_triple`

  - behavior

    - It transmits three kinds of topic messages; `/chatter`, `/slower_chatter` and `/slowest_chatter`. They have different transmission frequency.
    - `/slower_chatter` and `/slowest_chatter` are not transmitted if there is no corresponding subscription
    - `/chatter` is transmitted at frequency defined by `update_frequency`
    - `/slower_chatter` is transmitted at one half of frequency defined by `update_frequency`
    - `/slowest_chatter` is transmitted at one third of frequency defined by `update_frequency`

  - option
    - `update_frequency` (float)
      - specify output frequency of messages
      - default is 1.0
    - `use_transient_local` (boolean)
      - enable Transient Local of Publisher
      - default is false

#### run listener

Launch another terminal and execute a listener node with the following command.

```console
ros2 run simple_examples [listener node]
```

#### listener nodes list

Below is the list of listener nodes included in the `waitset_examples` package.

- `timer_listener_single`
  - behavior
    - It has a cyclic timer to execute a callback function in which a `/chatter` message is obtained from Subscription Queue after arrival of a message is checked with `rlcpp::WaitSet`
    - It print the obtained message by `RCLCPP_INFO`
  - option
    - `update_frequency` (float)
      - specify timer frequency
      - default is 1.0
    - `use_transient_local` (boolean)
      - enable Transient Local of Subscription
      - default is false
- `timer_listener_using_callback`
  - behavior
    - It has a cyclic timer to execute a callback function in which a `/chatter` message processed with the combination of `take_type_erased()` and `handle_message()` after arrival of a message is checked with `rlcpp::WaitSet`
    - It has the same behavior with `timer_listener_single`
  - option
    - `update_frequency` (float)
      - specify timer frequency
      - default is 1.0
    - `use_transient_local` (boolean)
      - enable Transient Local of Subscription
      - default is false
- `timer_listener_single_wait_some_period`
  - behavior
    - It has a cyclic timer to execute a callback function in which a `/chatter` message is obtained from Subscription Queue after arrival of a message is checked with `rlcpp::WaitSet`
    - Duration for timeout is given as the given as the argument of `wait`
    - It prints out string data in a incoming message if it receives the message by the duration
    - Otherwise it prints out that timeout occurs during waiting a message
  - option
    - `update_frequency` (float)
      - specify timer frequency
      - default is 1.0
    - `use_transient_local` (boolean)
      - enable Transient Local of Subscription
      - default is false
    - `waiting_time` (float)
      - specify timeout value
      - default is 4.0
- `timer_batch_listener_single`
  - behavior
    - It has a cyclic timer to execute a callback function in which a `/chatter` message is obtained from Subscription Queue until the queue becomes empty after after arrival of a message is checked with `rlcpp::WaitSet`
  - option
    - `update_frequency` (float)
      - specify timer frequency
      - default is 0.2
    - `use_transient_local` (boolean)
      - enable Transient Local of Subscription
      - default is false
    - `queue_size` (integer)
      - specify Subscription Queue size
      - default is 10
- `timer_listener_triple_sync`
  - behavior
    - It has a cyclic timer to execute a callback function in which `/chatter`, `/slower_chatter`, and `/slowest_chatter` messages are obtained from each Subscription Queue. If all of the three subscriptions have any message, they are taken at the same time
    - When all of the three subscriptions receive any message, they are taken from the subscriptions 
    - waitset is used to verify it
  - option
    - `update_frequency` (float)
      - specify timer frequency
      - default is 1.0
    - `use_transient_local` (boolean)
      - enable Transient Local of Subscription
      - default is false
- `timer_listener_triple_async`
  - behavior overview
    - driven by timer periodically to execute a callback function in which `/chatter`, `/slower_chatter`, and `/slowest_chatter` messages are obtained from each Subscription Queue
    - the messages are obtained if one or more messages are in each Subscription Queue of `/chatter`, `/slower_chatter`, and `/slowest_chatter`
    - the number of the messages is at most one from each Subscription Queue
    - waitset is used to verify it
  - option
    - `update_frequency` (float)
      - specify timer frequency
      - default is 1.0
    - `use_transient_local` (boolean)
      - enable Transient Local of Subscription
      - default is false
- `timer_listener_triple_separated_waitset`
  - behavior overview
    - driven by three timers periodically to execute a callback function in which `/chatter`, `/slower_chatter`, and `/slowest_chatter` messages are obtained from each Subscription Queue
    - the messages are obtained if one or more messages are in each Subscription Queue of `/chatter`, `/slower_chatter`, and `/slowest_chatter`
    - dedicated three waitsets are used to verify it unlike `timer_listener_triple_async`, in which one waitset is used
  - option
    - `update_frequency` (float)
      - specify timer frequency
      - default is 1.0
    - `use_transient_local` (boolean)
      - enable Transient Local of Subscription
      - default is false
- `timer_listener_triple_sync_intra`
  - behavior overview
    - similar to `timer_listener_triple_sync`
    - support intra-process communication unlike `timer_listener_triple_sync` in which only inter-process communication can be used
    - run the following command for this node to run
      - `ros2 launch waitset_examples talker_listener_triple_intra.launch.py use_intra_process_comms:=true`
      - if `use_intra_process_comms` is set to false, communication is performed through DDS
  - option
    - `update_frequency` (float)
      - specify timer frequency
      - default is 1.0
    - `use_transient_local` (boolean)
      - enable Transient Local of Subscription
      - default is false
- `timer_listener_twin_static`
  - behavior overview
    - It has a cyclic timer to execute a callback function in which `/chatter` and `/slower_chatter` messages are obtained from each Subscription Queue
    - `rclcpp::StaticWaitSet` is used instead of `rclcpp::WaitSet`
    - note that triggers can be registered to `rclcpp::StaticWaitSet` only at initialization unlike `rclcpp::WaitSet` to which triggers can be registered at any time
    - also refer to [examples/rclcpp/wait_set/src/static_wait_set.cpp at rolling · ros2/examples](https://github.com/ros2/examples/blob/rolling/rclcpp/wait_set/src/static_wait_set.cpp)
  - option
    - `update_frequency` (float)
      - specify timer frequency
      - default is 1.0
    - `use_transient_local` (boolean)
      - enable Transient Local of Subscription
      - default is false
- `timer_listener_twin_nested_waitset`
  - behavior overview
    - run two timers one of which is `timer_` and another is `much_slower_timer_`
    - `timer_` is invoked per specified frequency and execute its callback function
    - `much_slower_timer_` is invoked per one tenth of frequency of `timer_`
    - `much_slower_timer_` has its callback function but it is not registered to Executor
    - the callback function of `timer_` is executed periodically in which it is verified that `much_slower_timer_` has been invoked or not by waitset
    - if yes, execute the callback function of `much_slower_timer_` through `execute_callback()`
    - at that time it is verified that Subscription of `/slower_chatter` has been invoked or not by another waitset
  - option
    - `update_frequency` (float)
      - specify timer frequency
      - default is 1.0
    - `use_transient_local` (boolean)
      - enable Transient Local of Subscription
      - default is false
