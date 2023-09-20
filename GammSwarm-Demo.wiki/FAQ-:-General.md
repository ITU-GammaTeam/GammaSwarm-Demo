<a name='Name'/>

## Where does the name come from?

Naming things is [hard](http://martinfowler.com/bliki/TwoHardThings.html).

We wanted a name that would keep the theme of resilience, defense and fault tolerance while being short, easy to say and not already taken. We considered many words - synonyms, adjectives, animals - and mashups of them all, and Hystrix came out on top.

A Hystrix is an "[Old World porcupine](http://en.wikipedia.org/wiki/Hystrix)" with an [impressive defense mechanism](http://www.arkive.org/north-african-crested-porcupine/hystrix-cristata/video-11b.html). It's also (in my opinion) a cool name, only 2 syllables (I end up saying it a lot so this was important), looks nice written out and when Googling for it only found animals so seemed free from collision with other products.

And it allows for cool artistic interpretations such as this logo:

[[images/hystrix-logo.png]]


<a name='AtNetflix'/>

## How is this used at Netflix?

Netflix uses Hystrix in many applications, particularly its edge services such as the Netflix API. Tens of billions of thread-isolated and hundreds of billions of semaphore-isolated calls are executed via Hystrix every day at Netflix.

To learn more about how Hystrix is used and where it evolved from take a look at these blogs:

* [Making Netflix API More Resilient](http://techblog.netflix.com/2011/12/making-netflix-api-more-resilient.html)
* [Fault Tolerance in a High Volume, Distributed System](http://techblog.netflix.com/2012/02/fault-tolerance-in-high-volume.html)

Also, this slidedeck is from a presentation that goes into a little more detail about its usage on the Netflix API:

* [Performance and Fault Tolerance for the Netflix API](https://speakerdeck.com/benjchristensen/performance-and-fault-tolerance-for-the-netflix-api-august-2012)


<a name='Intrusive'/>

## Why is it so intrusive?

Common first reactions to Hystrix (even internally at Netflix when first introduced to teams) include:

- Why is this so intrusive?
- Why do I need to change my client libraries?
- Why is it a command pattern that requires wrapping libraries or network calls?
- Why not intercept calls at a lower level?

By design Hystrix intends to offer a clearly defined barrier of "host app" versus "dependency". Anything that goes over the network or can possibly trigger something that goes over the network is a possible source of failure or latency. Hystrix explicitly adds a layer between these points of failure. This is not only for functional reasons but also as a standard mechanism for communicating to users of that object that it is a "protected" resource.

When developing Hystrix at Netflix we specifically sought out transparent network calls and wrapped them in HystrixCommand implementations as on multiple occasions these were the cause of production outages.

Developers interact with a library that accesses resources over a network very differently than one that operates on in-memory data.

Thus, the addition of the Hystrix layer serves these purposes:

- Communicate resilience to anyone calling it.  
- Developers trust the object, can configure and monitor it and won't inadvertently add yet another wrapping layer by being unaware of hidden resiliency features.  
- Ability to execute synchronously (HystrixCommand.execute()) or asynchronously (HystrixCommand.queue()).  
- Ability to query a command after execution for state (fallback, errors, metrics, etc)  

The [[migration of a library|How-To-Use#wiki-MigratingLibrary]] to using Hystrix typically looks like this:

[[images/library-migration-to-hystrix-with-640.png]]

If a client invokes functionality via the service facade rather than the HystrixCommand they will not receive those benefits and functionality but they will still receive the fault tolerance - just transparently.

If you still feel strongly that you shouldn't have to modify libraries and add command objects then perhaps you can [[contribute an AOP module|FAQ#wiki-AOP]].


<a name='TransitiveDependencies'/>

## What about transitive dependencies?

Transitive dependencies and thus transitive calls to HystrixCommands are expected and okay.

It does not negate the benefit of visibility and communicating resilience if I interact with a HystrixCommand which then happens to invoke others – the trust is given by the single command being interacted with at the top of the call. Anything below that is being used with the scope of the initial protection.

The transitive commands provide modular fault tolerance for each piece of aggregate functionality required by the first command.

Also, because all command executions are logged for a request, metrics on transitive HystrixCommands are also exposed even though the caller may not have directly invoked it.

[[images/transitive-commands-640.png]]


<a name='Annotations'/>

## Can annotations be used?

Not as part of [hystrix-core](https://github.com/Netflix/Hystrix/tree/master/hystrix-core) functionality. It has been considered but not pursued. It is definitely a candidate for someone to implement as a [sub-module](https://github.com/Netflix/Hystrix/tree/master/hystrix-contrib).

The primary design principle this doesn't mesh very well with is that it makes the isolation barriers transparent (see [["Why is it so intrusive?"|FAQ#wiki-Intrusive]] for more reasoning on this). In other words, a consumer of a library would no longer see a HystrixCommand implementation with standard execute(), queue() and other functionality nor receive the communication of isolation and fault tolerance that is assumed when interacting with a HystrixCommand. They would just invoke a method and have no idea of whether it's isolated or not.

<a name='AOP'/>

## Why not use AOP?

AOP has been avoided as part of [hystrix-core](https://github.com/Netflix/Hystrix/tree/master/hystrix-core) functionality due to the non-obviousness of using it and the desire to stay away from bytecode manipulation. 

It also goes against the principles of Hystrix which prefer explicitly exposing access points to dependencies, networks and systems as points of possible failure (see [["Can annotations be used?"|FAQ#wiki-Annotations]] and [["Why is it so intrusive?"|FAQ#wiki-Intrusive]] for more reasoning on this).

However, there may be use cases where it's applicable and thus it is a candidate for someone to implement as a [sub-module](https://github.com/Netflix/Hystrix/tree/master/hystrix-contrib).

A related area where it may be useful is not for Hystrix command objects but for tracking drift – determining points of unwrapped network access that spring up over time.


<a name='InterceptNetwork'/>

## Why not just automatically intercept all network calls?

Network calls are too low in the stack to provide the needed business logic and granularity for fallback behavior and logical isolation.

Often a single network route via a cluster of loadbalancers will serve many different types of functionality that end up in several different HystrixCommands. 

Each HystrixCommand needs the ability to set different throughput constraints, timeout values and fallback strategies.

Also, all failure is not restricted to the network layer, the transport may occur fine but return data that the client fails to handle which then results in exceptions being thrown. A HystrixCommand not only wraps the network call but the processing of the response as well so errors are handled and fallback logic applied regardless of where the error occurred.

For example:

- single backend service (dozens of servers in a cluster) accepting reads and writes
- 3 types of "read" functionality, 1 "write" function
  - ABGetAllocationsForUser
  - ABGetAllocationsForCell
  - ABGetTestCells
  - ABSetAllocationForUser
- throughput on each of these varies so different sized thread-pools (or semaphores)
- write calls can fail independently of reads so reads are grouped on 1 thread-pool, writes on another
- reads have good fallback options for default behavior
- writes fail fast without fallback
- reads can be cached within a request
- writes can not be cached
- writing a SetAllocation knows to clear the cache of GetAllocations but doesn't need to on GetTestCells

In this example 4 separate HystrixCommands expose the functionality, wrap both network and client code execution and allow granular control of isolation and fallback behavior.

Resilience engineering becomes part of the library behavior and business logic. 

<a name='LoadBalancer'/>
## Why don't you just use a load-balancer?

Load balancers are themselves a network call and thus a point of failure. They are not trusted any more than any other network call.

Experiences by those involved in designing and building Hystrix include many production problems where load balancers were involved.

They also must be configured at the lowest common denominator such as:

- timeouts (if supported) set high for all possible calls via that route
- throughput for an entire cluster, not an application instance
- no business logic can be applied for fallbacks

A load balancer obviously plays a role in a highly available distributed system but it is serving a different set of needs than what Hystrix provides to an application instance.

Applications must be designed for resilience and not rely upon infrastructure – particularly infrastructure over the network.


<a name='Costs'/>

## What is the processing overhead of using Hystrix?

As a point of reference each Netflix API server executes around 350 thread-isolated and 5000+ semaphore-isolated HystrixCommand instances per second on 4-core Amazon EC2 servers.

The thread-isolated commands are separated by 40+ thread-pools with 5-20 threads in each and queue sizes of 5 or 10 fronting each thread-pool. Latency on thread-isolated command executions ranges from single-digit milliseconds to 1500ms+ at the 99th percentile for some commands.

The types of overhead Hystrix adds to an application are:

#### 1) Object Allocation

Each command invocation results in instantiating a new HystrixCommand object and associated objects within it for tracking state during the execution flow.

Most of this object allocation is quickly retrieved in young gen collection.

Netflix API servers executing over 5500 HystrixCommands per seconds per box on 4-core boxes and object allocation of HystrixCommands is marginal compared with that by the business functionality (such as string allocations for JSON and other serialization/deserialization of network communication).


#### 2) Concurrency

There are a lot of shared data structures in Hystrix, particularly around metrics.

Effort has been made to use non-blocking approaches to concurrency and use atomics rather than locks wherever possible (and where locks are used they are of the tryLock variety) to avoid causing threads to be put to sleep and affect throughput.

An area where threads are purposefully blocked is when request caching is used and multiple threads are waiting on a single network execution. In this case the cost of the command execution (typically a network call) outweighs the cost of having threads block and be rescheduled so a CountDownLatch is used to block waiting threads until the response is available.

The Future.get() behavior will also cause calling threads to block on a thread-isolated execution while the underlying run() method is invoked on a child thread.

Additionally the following types of functionality use atomic counters which under high contention have cost:

- rolling and cumulative counters
- rolling percentile calculations of latency
- semaphores around run() and getFallback() execution

To mitigate possible performance impact on highly concurrent systems (such as a 32-core box with thousands of executions per second) Hystrix uses early Java 8 libraries wherever possible to reduce contention on atomic counters: [LongAdder](../blob/master/hystrix-core/src/main/java/com/netflix/hystrix/util/LongAdder.java) and [LongMaxUpdater](../blob/master/hystrix-core/src/main/java/com/netflix/hystrix/util/LongMaxUpdater.java).


#### 3) Thread Execution (unless using semaphore isolation)

When using thread-isolation (the default and recommended behavior for HystrixCommands performing network access) each HystrixCommand execution results in a task being queued on a thread-pool for execution and a child thread performing the work.

This means there is the overhead of using another thread including the handoff, scheduling, execution and retrieving the value.


#### Measuring Cost

Hystrix measures the latency when executing the run() method on the child thread as well as the total end-to-end time on the parent thread to expose the cost of Hystrix overhead (threading, metrics, logging, circuit breaker, etc).

The following diagram represents one HystrixCommand being executed at 60 requests-per-second on a single API instance:

<a href="images/thread-cost-60rps-original.png">[[images/thread-cost-60rps-640.png]]</a>
_(Click for larger view)_

At the median (and lower) there is no cost.

At the 90th percentile there is a cost of 3ms.

At the 99th percentile there is a cost of 9ms. Note however that the increase in cost is far smaller than the increase in execution time of the separate thread (network request) which jumped from 2 to 28 whereas the cost jumped from 0 to 9.

This overhead at the 90th percentile and higher for circuits such as these has been deemed acceptable for most Netflix use cases for the benefits of resilience achieved.

For circuits that wrap very low latency requests (such as those primarily hitting in-memory caches) the overhead can be too high and in those cases we choose to use tryable semaphores which do not allow for timeouts but provide most of the resilience benefits without the overhead. The overhead in general though is small enough that we prefer the isolation benefits of a separate thread.

<a name='Async'/>

## What about asynchronous dependency calls?

These are supported as of Hystrix 1.4.
