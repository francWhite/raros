plugins {
  id("java")
  application
}

group = "io.github.francwhite"
version = "1.0-SNAPSHOT"

repositories {
  mavenCentral()
}

dependencies {
  testImplementation(platform("org.junit:junit-bom:5.9.1"))
  testImplementation("org.junit.jupiter:junit-jupiter")
  implementation(project(":raros-client"))
}

tasks.test {
  useJUnitPlatform()
}

application {
  mainClass.set("io.github.francwhite.raros.example.InteractiveExample")
}