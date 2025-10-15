package com.moniev.boids.core.MainEngine;

import java.util.ArrayList;

import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.moniev.boids.core.Boid.Boid;
import com.moniev.boids.core.Vector.Vector;

public class OctreeNode {
  public final int boidsLimit, depthLimit;

  public final float minX, minY, minZ;
  public final float maxX, maxY, maxZ;

  private final float size, halfSize, quarterSize;

  public final Vector center;

  private Octree tree;
  public OctreeNode parent;
  public OctreeNode[] children;
  public ArrayList<Boid> boids;

  public boolean isLeaf, isBorder;

  public final ModelInstance modelInstance;

  public OctreeNode(Vector center, float size, int boidsLimit, int depthLimit, OctreeNode parent, Octree tree) {
    this.tree = tree;
    this.parent = parent;
    this.boidsLimit = boidsLimit;
    this.depthLimit = depthLimit;

    this.center = center;
    this.size = size;

    this.children = new OctreeNode[8];
    this.boids = new ArrayList<>();

    this.isLeaf = true;

    this.halfSize = this.size / 2f;
    this.quarterSize = this.size / 4f;

    this.minX = center.x - halfSize;
    this.minY = center.y - halfSize;
    this.minZ = center.z - halfSize;

    this.maxX = center.x + halfSize;
    this.maxY = center.y + halfSize;
    this.maxZ = center.z + halfSize;

    this.modelInstance = new ModelInstance(tree.engine.getNodeModel());
    this.modelInstance.transform.setToTranslation(center.x, center.y, center.z);
    this.modelInstance.transform.scl(size);
    this.isBorder = isBorder();
  }

  private boolean isBorder() {
    return (minX <= tree.minX || maxX >= tree.maxX ||
        minY <= tree.minY || maxY >= tree.maxY ||
        minZ <= tree.minZ || maxZ >= tree.maxZ);
  }

  public boolean isEmpty() {
    return boids.isEmpty();
  }

  public void insert(Boid boid) {
    if (isLeaf) {
      boids.add(boid);

      if (boids.size() > boidsLimit && depthLimit > 0) {
        subdivide();
        redistributeBoids();
      }
    } else {
      if (children[0] == null) {
        subdivide();
      }
      int i = getChildIndex(boid.position);
      children[i].insert(boid);
    }
  }

  public void subdivide() {
    for (int i = 0; i < 8; i++) {
      float xOffSet = ((i & 1) == 0) ? -quarterSize : quarterSize;
      float yOffSet = ((i & 2) == 0) ? -quarterSize : quarterSize;
      float zOffSet = ((i & 4) == 0) ? -quarterSize : quarterSize;

      Vector childCenter = new Vector(
          center.x + xOffSet,
          center.y + yOffSet,
          center.z + zOffSet);
      children[i] = new OctreeNode(childCenter, halfSize, boidsLimit, depthLimit - 1, this, this.tree);
    }
    isLeaf = false;
  }

  public void redistributeBoids() {
    ArrayList<Boid> temp = new ArrayList<>(boids);
    boids.clear();

    for (Boid boid : temp) {
      int i = getChildIndex(boid.position);
      children[i].insert(boid);
    }
  }

  public int getChildIndex(Vector position) {
    int index = 0;
    if (position.x >= center.x)
      index |= 1;
    if (position.y >= center.y)
      index |= 2;
    if (position.z >= center.z)
      index |= 4;
    return index;
  }

  public boolean isAdjacent(OctreeNode other) {
    if (this == other) {
      return false;
    }

    final float epsilon = 0.001f;

    boolean overlapY = (this.maxY > other.minY) && (this.minY < other.maxY);
    boolean overlapZ = (this.maxZ > other.minZ) && (this.minZ < other.maxZ);

    if (Math.abs(this.maxX - other.minX) < epsilon && overlapY && overlapZ)
      return true;
    if (Math.abs(this.minX - other.maxX) < epsilon && overlapY && overlapZ)
      return true;

    boolean overlapX = (this.maxX > other.minX) && (this.minX < other.maxX);

    if (Math.abs(this.maxY - other.minY) < epsilon && overlapX && overlapZ)
      return true;
    if (Math.abs(this.minY - other.maxY) < epsilon && overlapX && overlapZ)
      return true;

    if (Math.abs(this.maxZ - other.minZ) < epsilon && overlapX && overlapY)
      return true;
    if (Math.abs(this.minZ - other.maxZ) < epsilon && overlapX && overlapY)
      return true;

    return false;
  }

  public boolean isNearBorder(Boid boid) {
    float margin = 1.25f;

    return (boid.position.x - margin < minX || boid.position.x + margin > maxX ||
        boid.position.y - margin < minY || boid.position.y + margin > maxY ||
        boid.position.z - margin < minZ || boid.position.z + margin > maxZ);
  }

  public ArrayList<OctreeNode> getAdjacentNodes() {
    ArrayList<OctreeNode> adjacentNodes = new ArrayList<>();
    OctreeNode root = this.tree.root;

    float epsilon = 0.01f;
    float searchMinX = this.minX - epsilon;
    float searchMaxX = this.maxX + epsilon;
    float searchMinY = this.minY - epsilon;
    float searchMaxY = this.maxY + epsilon;
    float searchMinZ = this.minZ - epsilon;
    float searchMaxZ = this.maxZ + epsilon;

    findAdjacentNodesRecursive(root, adjacentNodes, searchMinX, searchMaxX, searchMinY, searchMaxY, searchMinZ,
        searchMaxZ);
    return adjacentNodes;
  }

  private void findAdjacentNodesRecursive(OctreeNode currentNode, ArrayList<OctreeNode> result,
      float sMinX, float sMaxX, float sMinY, float sMaxY, float sMinZ, float sMaxZ) {

    boolean intersects = (currentNode.maxX > sMinX && currentNode.minX < sMaxX) &&
        (currentNode.maxY > sMinY && currentNode.minY < sMaxY) &&
        (currentNode.maxZ > sMinZ && currentNode.minZ < sMaxZ);

    if (!intersects) {
      return;
    }

    if (currentNode.isLeaf) {
      if (currentNode != this) {
        result.add(currentNode);
      }
    } else {
      for (OctreeNode child : currentNode.children) {
        if (child != null) {
          findAdjacentNodesRecursive(child, result, sMinX, sMaxX, sMinY, sMaxY, sMinZ, sMaxZ);
        }
      }
    }
  }
}
