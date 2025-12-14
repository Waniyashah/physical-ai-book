---
sidebar_position: 2
---

# Chapter 2: ROS 2 Services & Actions

In the previous chapter, we learned about ROS 2 topics, which provide a one-way, asynchronous communication channel. In this chapter, we will explore two other important communication mechanisms in ROS 2: **Services** and **Actions**.

## ROS 2 Services: Request/Response Communication

A ROS 2 **Service** provides a two-way, synchronous communication channel. Services are based on a request/response model, where a **client** node sends a request to a **server** node and waits for a response. This is useful for tasks that require a direct confirmation or a result, such as querying the state of a sensor or triggering a specific robot behavior.

### How it Works

1.  A **client** node creates a request message and sends it to a service.
2.  A **server** node receives the request, performs a task, and creates a response message.
3.  The server sends the response back to the client.

Services are ideal for quick, non-blocking tasks that have a clear beginning and end.

## ROS 2 Actions: Long-Running, Asynchronous Tasks

A ROS 2 **Action** is used for long-running, asynchronous tasks that provide feedback during their execution. Actions are more complex than services and are designed for tasks that may take a significant amount of time to complete, such as navigating to a goal or executing a multi-step manipulation task.

### How it Works

1.  An **action client** sends a goal to an **action server**.
2.  The action server accepts the goal and begins executing the task.
3.  The action server provides regular **feedback** to the action client about the progress of the task.
4.  When the task is complete, the action server sends a **result** to the action client.

The action client can also cancel the goal at any time. This makes actions suitable for complex behaviors that require monitoring and potential preemption.

## Choosing the Right Communication Mechanism

-   **Topics**: Use for continuous data streams where the sender doesn't need to know if the data was received (e.g., sensor data, robot state).
-   **Services**: Use for quick, transactional request/response interactions where a confirmation is needed (e.g., getting a parameter, triggering a simple behavior).
-   **Actions**: Use for long-running, asynchronous tasks that require feedback and the ability to be canceled (e.g., navigation, manipulation).

In the next chapter, we will learn how to put all these concepts together by building our own ROS 2 package.
