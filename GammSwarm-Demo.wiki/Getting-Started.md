## Contents
1. <a href="#binaries">Getting Binaries</a>
1. <a href="#hello">&ldquo;Hello World!&rdquo;</a>
1. <a href="#building">Building</a>

<a name="binaries" />

## Getting Binaries

Binaries and dependency information for Maven, Ivy, Gradle and others can be found at [http://search.maven.org](http://search.maven.org/#search%7Cga%7C1%7Cg%3A%22com.netflix.hystrix%22%20AND%20a%3A%22hystrix-core%22).

Example for Maven:

```xml
<dependency>
    <groupId>com.netflix.hystrix</groupId>
    <artifactId>hystrix-core</artifactId>
    <version>x.y.z</version>
</dependency>
```
and for Ivy:

```xml
<dependency org="com.netflix.hystrix" name="hystrix-core" rev="x.y.z" />
```

If you need to download the jars instead of using a build system, create a Maven pom file like this with the desired version:

```xml
<?xml version="1.0"?>
<project xmlns="http://maven.apache.org/POM/4.0.0" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="http://maven.apache.org/POM/4.0.0 http://maven.apache.org/xsd/maven-4.0.0.xsd">
	<modelVersion>4.0.0</modelVersion>
	<groupId>com.netflix.hystrix.download</groupId>
	<artifactId>hystrix-download</artifactId>
	<version>1.0-SNAPSHOT</version>
	<name>Simple POM to download hystrix-core and dependencies</name>
	<url>http://github.com/Netflix/Hystrix</url>
	<dependencies>
		<dependency>
			<groupId>com.netflix.hystrix</groupId>
			<artifactId>hystrix-core</artifactId>
			<version>x.y.z</version>
			<scope/>
		</dependency>
	</dependencies>
</project>
```

Then execute:

```
mvn -f download-hystrix-pom.xml dependency:copy-dependencies
```

It will download hystrix-core-*.jar and its dependencies into ./target/dependency/.

You need Java 6 or later.

<a name="hello" />

## Hello World!

The simplest use of Hystrix is as follows:

```java
public class CommandHelloWorld extends HystrixCommand<String> {

    private final String name;

    public CommandHelloWorld(String name) {
        super(HystrixCommandGroupKey.Factory.asKey("ExampleGroup"));
        this.name = name;
    }

    @Override
    protected String run() {
        // a real example would do work like a network call here
        return "Hello " + name + "!";
    }
}
```
[View Source](../blob/master/hystrix-examples/src/main/java/com/netflix/hystrix/examples/basic/CommandHelloWorld.java)

This command could be used like this:

```java
String s = new CommandHelloWorld("Bob").execute();
Future<String> s = new CommandHelloWorld("Bob").queue();
Observable<String> s = new CommandHelloWorld("Bob").observe();
```

More examples and information can be found in the [[How To Use]] section.

Example source code can be found in the [hystrix-examples](../tree/master/hystrix-examples/src/main/java/com/netflix/hystrix/examples) module.

<a name="building" />

## Building

To checkout the source and build:

```
$ git clone git@github.com:Netflix/Hystrix.git
$ cd Hystrix/
$ ./gradlew build
```

To do a clean build:

```
$ ./gradlew clean build
```

A build should look like this:

```
$ ./gradlew build
:hystrix-core:compileJava
:hystrix-core:processResources UP-TO-DATE
:hystrix-core:classes
:hystrix-core:jar
:hystrix-core:sourcesJar
:hystrix-core:signArchives SKIPPED
:hystrix-core:assemble
:hystrix-core:licenseMain UP-TO-DATE
:hystrix-core:licenseTest UP-TO-DATE
:hystrix-core:compileTestJava
:hystrix-core:processTestResources UP-TO-DATE
:hystrix-core:testClasses
:hystrix-core:test
:hystrix-core:check
:hystrix-core:build
:hystrix-examples:compileJava
:hystrix-examples:processResources UP-TO-DATE
:hystrix-examples:classes
:hystrix-examples:jar
:hystrix-examples:sourcesJar
:hystrix-examples:signArchives SKIPPED
:hystrix-examples:assemble
:hystrix-examples:licenseMain UP-TO-DATE
:hystrix-examples:licenseTest UP-TO-DATE
:hystrix-examples:compileTestJava
:hystrix-examples:processTestResources UP-TO-DATE
:hystrix-examples:testClasses
:hystrix-examples:test
:hystrix-examples:check
:hystrix-examples:build

BUILD SUCCESSFUL

Total time: 30.758 secs
```

On a clean build you will see the unit tests be run and look something like this:

```
> Building > :hystrix-core:test > 147 tests completed
```