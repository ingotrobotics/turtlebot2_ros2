// (c) 2024 Ingot Robotics
//
// Jenkinsfile to lint, build, and test turtlebot2_ros2 repo

// Supported ROS 2 versions are given in the `rosVersions` list
def rosVersions = ['jazzy', 'iron', 'humble']

// For each ROS 2 version, build from the official docker image and the OSRF
// desktop image
def baseImages = []
rosVersions.each {
    baseImages.add( 'ros:' + it )
    baseImages.add( 'osrf/ros:' + it + '-desktop')
}

// Translate the Git branch name into a format that is safe to use in Docker tags
// https://docs.docker.com/reference/cli/docker/image/tag/#extended-description,
// 1. Do not add if branch is "main" or "master"
// 2. Consider only after last `/`
def branchTag = ""
if (env.BRANCH_NAME != "main" || env.BRANCH_NAME != "master" ) {
    branchTag = '-' + env.BRANCH_NAME.tokenize('/').last()
// 2. Convert allowed Git characters that are not allowed by Docker
// TODO
}

// If Ingot Robotics shared library is not available on the Jenkins instance,
// skip local registry steps
localRegistryExists = false
try {
    library('ingot-robotics')
    localRegistryExists = true
}
catch (e) {
    localRegistryExists = false
    print e
}


pipeline {
    agent none
    stages {
        /* TODO
        stage ("test ROS in docker container")
        */
        stage ("lint dockerfile") {
            agent {
                docker {
                    image 'hadolint/hadolint:latest-debian'
                }
            }
            steps {
                sh 'hadolint --ignore DL3006 --ignore DL3008 turtlebot2_ros2.dockerfile | tee -a hadolint_lint.txt'
            }
            post {
                always {
                    archiveArtifacts 'hadolint_lint.txt'
                }
            }
        }
        stage ("build containers") {
            environment {
                DOCKER_BUILDKIT = '1'
            }
            steps {
                script {
                    def parallelStages = baseImages.collectEntries {
                        ["$it$branchTag" : {
                            node {
                                stage("tag: ${it}" + "${branchTag}") {
                                    def tag = it.tokenize(':').last() + branchTag
                                    echo "Building ${it}${branchTag} ..."
                                    checkout scm
                                    sh "docker build \
                                        -t ingot/turtlebot2-ros2:${tag} \
                                        -f turtlebot2_ros2.dockerfile \
                                        --build-arg from_image=${it} \
                                        --build-arg parallel_jobs=2 ."
                                }
                            }
                        }]
                    }
                    parallel parallelStages
                }
            }
        }
        /* TODO
        stage ("test built code in container")
        */
        stage ("push to local registry") {
            agent any
            when {
                expression{ localRegistryExists==true }
            }
            steps {
                pushToLocalRegistry( "ingot/turtlebot2-ros2" )
            }
        }
    }
}
