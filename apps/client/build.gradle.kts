import org.jreleaser.model.Active

plugins {
  `java-library`
  `maven-publish`
  signing
  id("org.jreleaser") version "1.9.0"
}

group = "io.github.francwhite"
version = "1.0.0"

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

java {
  withSourcesJar()
  withJavadocJar()
}

publishing {
  publications {
    create<MavenPublication>("maven") {
      from(components["java"])
      pom {
        packaging = "jar"
        name = "Raros client library"
        description = "Client library to interact with raros"
        url = "https://github.com/francWhite/raros"
        licenses {
          license {
            name = "MIT License"
            url = "https://opensource.org/licenses/MIT"
          }
        }
        developers {
          developer {
            id = "francWhite"
            name = "Francisco Metzger"
            email = "francisco.metzger@gmail.com"
            url = "https://github.com/francWhite"
          }
        }
        scm {
          connection = "scm:git:git://github.com/francWhite/raros.git"
          developerConnection = "scm:git:ssh://github.com/francWhite/raros.git"
          url = "https://github.com/francWhite/raros"
        }
      }
    }
  }
  repositories {
    maven {
      url = layout.buildDirectory.dir("staging-deploy").get().asFile.toURI()
    }
  }
}

signing {
  sign(publishing.publications["maven"])
}

jreleaser {
  project {
    copyright.set("Francisco Metzger")
  }
  gitRootSearch.set(true)
  signing {
    active.set(Active.ALWAYS)
    armored.set(true)
  }
  deploy {
    maven {
      nexus2 {
        create("maven-central") {
          active.set(Active.ALWAYS)
          url.set("https://s01.oss.sonatype.org/service/local")
          closeRepository.set(false)
          releaseRepository.set(false)
          stagingRepositories.add("build/staging-deploy")
        }
      }
    }
  }
}