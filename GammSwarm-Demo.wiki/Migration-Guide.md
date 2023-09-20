## Changes from 1.3.x to 1.4.x
* Addition of HystrixObservableCommand.  See [#321](https://github.com/Netflix/Hystrix/issues/321) for full description of design
* Timeouts now apply to semaphore-isolated commands as well as thread-isolated commands.  Before 1.4.x, semaphore-isolated commands could not timeout.  They now have a timeout registered on another (HystrixTimer) thread, which triggers the timeout flow.  If you use semaphore-isolated commands, they will now see timeouts.  As all HystrixCommands have a [default timeout](https://github.com/Netflix/Hystrix/wiki/Configuration#execution.isolation.thread.timeoutInMilliseconds), this potentially affects all semaphore-isolated commands.
* Timeouts now fire on `HystrixCommand.queue()`, even if the caller never calls `get()` on the resulting Future.  Before 1.4.x, only calls to `get()` triggered the timeout mechanism to take effect.
* Execution hooks changed to more closely match the Observable model.  The hooks from 1.3 still exist and still get invoked, but are now deprecated. See https://github.com/Netflix/Hystrix/issues/682 for all the details
* Execution Hooks that generate BadRequestExceptions now behave the same as if a command generated it (i.e. fallback/metrics are skipped)
* Addition of metrics
  * command metrics for bad request, max concurrent requests, number of emissions, number of fallback emissions
  * thread pool metrics for number of thread pool-rejections
  * collapser metrics for number of commands collapsed, number of batches, size of batches, size of shards
* Added EMIT and FALLBACK_EMIT event types that will show up in the HystrixRequestLog for HystrixObservableCommands.  These show how many items are in the streams of data produced by the commands
* ~~Upgraded to Java7~~ (1.4.0/1.4.1/1.4.2 were built with Java 7, but 1.4.3+ rolled back to Java 6)
* Upgraded to RxJava 1.0.x (1.0.7 currently)



## (Proposed) changes from 1.4.x to 1.5.x
* Remove deprecated hooks [#684](https://github.com/Netflix/Hystrix/issues/684)
* Scalar asynchronous command [#602](https://github.com/Netflix/Hystrix/issues/602)