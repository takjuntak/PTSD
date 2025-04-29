
# backend/ptsd/Dockerfile

FROM amazoncorretto:17
ARG JAR_FILE=./build/libs/monthlyzip-0.0.1-SNAPSHOT.jar
ARG PROFILE
ENV SPRING_PROFILES_ACTIVE=${PROFILE}
WORKDIR /app
COPY ${JAR_FILE} app.jar
COPY src/main/resources/application-secret.yml /app/application-secret.yml
ENTRYPOINT ["java", "-Dspring.profiles.active=${SPRING_PROFILES_ACTIVE}", "-jar", "app.jar"]