plugins {
  id("java")
  application
}

group = "ch.hslu.raros.example"
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
  mainClass.set("ch.hslu.raros.example.App")
}