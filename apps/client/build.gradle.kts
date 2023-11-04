plugins {
  id("java")
}

group = "ch.hslu.raros.client"
version = "1.0-SNAPSHOT"

repositories {
  mavenCentral()
}

dependencies {
  testImplementation(platform("org.junit:junit-bom:5.9.1"))
  testImplementation("org.junit.jupiter:junit-jupiter")
  implementation("com.fasterxml.jackson.core:jackson-databind:2.15.3")
}

tasks.test {
  useJUnitPlatform()
}