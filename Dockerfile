FROM ubuntu:focal

ARG WPILIB_URL=https://github.com/wpilibsuite/allwpilib/releases/download/v2022.4.1/WPILib_Linux-2022.4.1.tar.gz

# Install wget, set the working directory, and install WPILib
RUN apt-get update && apt-get install -y wget
WORKDIR /frc4488
RUN wget -q $WPILIB_URL && tar -xzf WPILib_Linux-*.tar.gz && tar -xzf WPILib_Linux-*/WPILib_Linux-*-artifacts.tar.gz && rm -rf WPILib_Linux-*
ENV JAVA_HOME /frc4488/jdk

# Install Gradle and download dependencies (separate from robot code for better caching)
WORKDIR /gradle
COPY build.gradle gradlew settings.gradle ./
COPY vendordeps ./vendordeps
COPY gradle ./gradle
COPY .wpilib ./.wpilib
RUN ./gradlew --no-daemon build

# Copy over and build the robot code
RUN mkdir -p /frc4488/FRC-Robot/Robot
COPY . /frc4488/FRC-Robot/Robot
WORKDIR /frc4488/FRC-Robot/Robot
RUN ./gradlew --no-daemon build

# Set the container to run the unit tests
ENTRYPOINT ["./gradlew", "--no-daemon", "--rerun-tasks", "test"]
