package com.moniev.boids.core.MainEngine;

import java.util.ArrayList;

import com.badlogic.gdx.graphics.g3d.ModelBatch;
import com.badlogic.gdx.graphics.g3d.utils.ModelBuilder;
import com.moniev.boids.core.Boid.Boid;
import com.moniev.boids.core.Vector.Vector;

public class Octree {
  public OctreeNode root;

  public final int maxDepth = 8;
  public int totalDepth = 1;

  public final float stepDt;
  public final Vector center;
  public final float minX, minY, minZ;
  public final float maxX, maxY, maxZ;
  public final Engine engine;
  private final float maxForce = 20f;
  private final float minForce = 1f;
  private final float distanceBetween = 26f;
  private final float alignmentForce = 1.2f;
  private final float cohesionForce = 1.1f;
  private final float separationForce = 1.7f;

  public Octree(Vector center, int size, int boidsLimit, int threads, ModelBuilder modelBuilder, float stepDt,
      Engine engine) {
    this.engine = engine;
    this.center = center;
    this.minX = center.x - size;
    this.minY = center.y - size;
    this.minZ = center.z - size;

    this.maxX = center.x + size;
    this.maxY = center.y + size;
    this.maxZ = center.z + size;

    this.stepDt = stepDt;

    this.root = new OctreeNode(center, size, boidsLimit, maxDepth, null, this);

  }

  private Vector limitForce(Vector vector) {
    float length = vector.length();

    if (length == 0) {
      return new Vector(0, 0, 0);
    }

    if (length > maxForce) {
      return vector.normalize().multiply(maxForce);
    } else if (length < minForce) {
      return vector.normalize().multiply(minForce);
    }

    return vector;
  }

  public int countBoids(OctreeNode root) {
    if (root == null)
      return 0;

    int count = 0;
    if (root.isLeaf) {
      count += root.boids.size();
    } else {
      for (OctreeNode child : root.children) {
        count += countBoids(child);
      }
    }

    return count;
  }

  public ArrayList<OctreeNode> getLeafs() {
    ArrayList<OctreeNode> leafs = new ArrayList<>();
    collectLeafs(root, leafs);
    return leafs;
  }

  public void collectLeafs(OctreeNode node, ArrayList<OctreeNode> leafNodes) {
    if (node == null)
      return;

    if (node.isLeaf) {
      leafNodes.add(node);
    } else {
      for (OctreeNode child : node.children) {
        collectLeafs(child, leafNodes);
      }
    }
  }

  public void addBoid(Boid boid) {
    root.insert(boid);
  }

  public void updateBoids(OctreeNode root, float subStepDt) {
    if (root == null)
      return;

    if (root.isLeaf) {
      for (Boid boid : root.boids) {
        boid.update(subStepDt);
      }
    } else {
      for (OctreeNode node : root.children) {
        updateBoids(node, subStepDt);
      }
    }
  }

  public void calculateForces(Boid boid) {
    OctreeNode currentNode = findTargetNode(this.root, boid);
    ArrayList<OctreeNode> nodesToSearch = currentNode.getAdjacentNodes();
    nodesToSearch.add(currentNode);

    Vector totalAlignment = new Vector(0, 0, 0);
    Vector totalCohesion = new Vector(0, 0, 0);
    Vector totalSeparation = new Vector(0, 0, 0);

    for (OctreeNode node : nodesToSearch) {
      totalAlignment.add(calculateAlignment(boid, node));
      totalCohesion.add(calculateCohesion(boid, node));
      totalSeparation.add(calculateSeparation(boid, node));
    }

    boid.acceleration.set(0f, 0f, 0f);
    boid.accelerate(limitForce(totalAlignment.substract(boid.getVelocity(engine.subStepDt)).multiply(alignmentForce)));
    boid.accelerate(limitForce(totalCohesion.substract(boid.position).multiply(cohesionForce)));
    boid.accelerate(limitForce(totalSeparation.multiply(separationForce)));
  }

  public ArrayList<Boid> getBorderBoids(ArrayList<OctreeNode> adjacentNodes) {
    ArrayList<Boid> borderBoids = new ArrayList<>();
    for (OctreeNode node : adjacentNodes) {
      for (Boid boid : node.boids) {
        if (node.isNearBorder(boid)) {
          borderBoids.add(boid);
        }
      }
    }
    return borderBoids;
  }

  public ArrayList<OctreeNode> getBorderNodes() {
    ArrayList<OctreeNode> borderNodes = new ArrayList<>();
    findBorderNodes(root, borderNodes);
    return borderNodes;
  }

  private void findBorderNodes(OctreeNode root, ArrayList<OctreeNode> borderNodes) {
    if (root == null)
      return;

    if (root.isLeaf && root.isBorder) {
      borderNodes.add(root);
    }

    for (OctreeNode node : root.children) {
      findBorderNodes(node, borderNodes);
    }
  }

  public void renderTree(ModelBatch modelBatch, OctreeNode root) {
    if (root == null)
      return;

    if (root.isLeaf) {
      modelBatch.render(root.modelInstance);
    } else {
      for (OctreeNode node : root.children) {
        renderTree(modelBatch, node);
      }
    }
  }

  public Vector calculateCohesion(Boid boid, OctreeNode node) {
    Vector cohesion = new Vector(0, 0, 0);
    int neighborsCount = 0;

    for (Boid other : node.boids) {
      if (other != boid) {
        float distance = boid.position.distance(other.position);
        if (distance > 0 && distance <= distanceBetween) {
          cohesion = cohesion.add(other.position);
          neighborsCount++;
        }
      }
    }

    if (neighborsCount > 0) {
      cohesion = cohesion.subdivide(neighborsCount).substract(boid.position);
      return limitForce(cohesion);
    }

    return new Vector(0, 0, 0);
  }

  public Vector calculateAlignment(Boid boid, OctreeNode node) {
    Vector alignment = new Vector(0, 0, 0);
    int neighborsCount = 0;

    for (Boid other : node.boids) {
      if (other != boid) {
        float distance = boid.position.distance(other.position);
        if (distance > 0 && distance <= distanceBetween) {
          alignment = alignment.add(other.getVelocity(engine.subStepDt));
          neighborsCount++;
        }
      }
    }

    if (neighborsCount > 0) {
      alignment = alignment.subdivide(neighborsCount).substract(boid.getVelocity(engine.subStepDt));
      return limitForce(alignment);
    }

    return new Vector(0, 0, 0);
  }

  public Vector calculateSeparation(Boid boid, OctreeNode node) {
    Vector separation = new Vector(0, 0, 0);

    for (Boid other : node.boids) {
      if (other != boid) {
        float distance = boid.position.distance(other.position);
        if (distance > 0 && distance <= distanceBetween) {
          Vector diff = boid.position.substract(other.position);
          float scale = 1.0f / (distance * distance);
          separation = separation.add(diff.multiply(scale));
        }
      }
    }

    return limitForce(separation);
  }

  public OctreeNode findTargetNode(OctreeNode root, Boid boid) {
    if (root.isLeaf) {
      return root;
    }
    int i = root.getChildIndex(boid.position);

    if (root.children[i] == null) {
      return root;
    }

    return findTargetNode(root.children[i], boid);
  }
}
