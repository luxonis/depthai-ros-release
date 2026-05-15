#!/bin/bash
set -e

BRANCH=$1
REGISTRY=$2
COMMIT_ID=$3
PULL_REQUEST=$4
TAG=$5

: "${PULL_REQUEST:="false"}"  # Fallback to false if not set or passed


if [ -z "$BRANCH" ] || [ -z "$REGISTRY" ] || [ -z "$COMMIT_ID" ] || [ -z "$PULL_REQUEST" ] || [ -z "$TAG" ]; then
  echo "Usage: $0 <branch> <registry> <commit_id> <is_pipeline_pull_request> <tag>"
  exit 1
fi

REPO_NAME="depthai-ros-hil"

IMAGE_NAME="${REPO_NAME}:${TAG}"
FULL_IMAGE_NAME="${REGISTRY}/${IMAGE_NAME}"

echo "Checking for existing image: $FULL_IMAGE_NAME"

# Check if image exists remotely
if curl --silent --fail "http://${REGISTRY}/v2/${REPO_NAME}/manifests/${TAG}" \
    -H "Accept: application/vnd.docker.distribution.manifest.v2+json" > /dev/null; then
  echo "✅ Remote image ${FULL_IMAGE_NAME} already exists. Skipping build and push."
  exit 0
fi

# Check if image exists locally
if docker images --format '{{.Repository}}:{{.Tag}}' | grep -q "^${IMAGE_NAME}$"; then
  echo "✅ Local image ${IMAGE_NAME} already exists. Skipping build."
else
  # Build the image
    echo "🔨 Building image ${IMAGE_NAME}..."
    docker build -t "${IMAGE_NAME}" -f ./Dockerfile . --build-arg BUILD_TESTS="1"
fi

# Push the image
echo "🚀 Tagging and pushing image to ${REGISTRY}..."
docker tag "${IMAGE_NAME}" "${FULL_IMAGE_NAME}"
docker push "${FULL_IMAGE_NAME}"
echo "✅ Done."
