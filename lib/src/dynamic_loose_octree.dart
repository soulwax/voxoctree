import 'dart:math' as math;

import 'frustum_cull.dart';
import 'math.dart';

/// Spatial entry stored inside [DynamicLooseOctree].
final class OctreeEntry<T> {
  final int id;
  final T item;
  OctreeAabb bounds;

  OctreeEntry({
    required this.id,
    required this.item,
    required this.bounds,
  });
}

final class _LooseNode<T> {
  final OctreeAabb tightBounds;
  final OctreeAabb looseBounds;
  final int depth;
  final List<OctreeEntry<T>> entries = [];
  List<_LooseNode<T>>? children;

  _LooseNode({
    required this.tightBounds,
    required this.looseBounds,
    required this.depth,
  });
}

/// Loose 3D octree supporting O(log N) dynamic insert/remove/update & hierarchical frustum culling.
final class DynamicLooseOctree<T> {
  final int maxDepth;
  final int splitThreshold;
  final double looseness;

  late final _LooseNode<T> _root;
  final Map<int, _LooseNode<T>> _entryNode = {};
  final Map<int, OctreeEntry<T>> _entriesById = {};

  DynamicLooseOctree({
    OctreeVec3 worldCenter = OctreeVec3.zero,
    double halfExtent = 128.0,
    this.maxDepth = 5,
    this.splitThreshold = 8,
    this.looseness = 2.0,
  }) {
    final he = math.max(1.0, halfExtent);
    final tight = OctreeAabb(
      OctreeVec3(worldCenter.x - he, worldCenter.y - he, worldCenter.z - he),
      OctreeVec3(worldCenter.x + he, worldCenter.y + he, worldCenter.z + he),
    );
    final looseHe = he * math.max(1.0, looseness);
    final loose = OctreeAabb(
      OctreeVec3(worldCenter.x - looseHe, worldCenter.y - looseHe, worldCenter.z - looseHe),
      OctreeVec3(worldCenter.x + looseHe, worldCenter.y + looseHe, worldCenter.z + looseHe),
    );
    _root = _LooseNode<T>(tightBounds: tight, looseBounds: loose, depth: 0);
  }

  int get count => _entriesById.length;
  int get nodeCount => _countNodes(_root);
  OctreeAabb get rootBounds => _root.looseBounds;

  int _countNodes(_LooseNode<T> node) {
    var c = 1;
    final ch = node.children;
    if (ch != null) {
      for (final child in ch) {
        c += _countNodes(child);
      }
    }
    return c;
  }

  void _ensureChildren(_LooseNode<T> node) {
    if (node.children != null) return;
    final c = node.tightBounds.center;
    final h = node.tightBounds.halfExtents * 0.5;
    final lh = h * math.max(1.0, looseness);
    final list = <_LooseNode<T>>[];
    for (var ox = -1; ox <= 1; ox += 2) {
      for (var oy = -1; oy <= 1; oy += 2) {
        for (var oz = -1; oz <= 1; oz += 2) {
          final cc = OctreeVec3(c.x + ox * h.x, c.y + oy * h.y, c.z + oz * h.z);
          list.add(
            _LooseNode<T>(
              tightBounds: OctreeAabb(cc - h, cc + h),
              looseBounds: OctreeAabb(cc - lh, cc + lh),
              depth: node.depth + 1,
            ),
          );
        }
      }
    }
    node.children = list;
  }

  bool _insertIntoNode(_LooseNode<T> node, OctreeEntry<T> entry) {
    if (node.depth < maxDepth &&
        (node.children != null || node.entries.length >= splitThreshold)) {
      _ensureChildren(node);
      final center = entry.bounds.center;
      for (final child in node.children!) {
        final tb = child.tightBounds;
        final centerInChild = center.x >= tb.min.x &&
            center.x <= tb.max.x &&
            center.y >= tb.min.y &&
            center.y <= tb.max.y &&
            center.z >= tb.min.z &&
            center.z <= tb.max.z;
        if (centerInChild && child.looseBounds.containsAabb(entry.bounds)) {
          return _insertIntoNode(child, entry);
        }
      }
    }
    node.entries.add(entry);
    _entryNode[entry.id] = node;
    _entriesById[entry.id] = entry;
    return true;
  }

  void insertOrUpdate(int id, T item, OctreeAabb bounds) {
    remove(id);
    final entry = OctreeEntry<T>(id: id, item: item, bounds: bounds);
    _insertIntoNode(_root, entry);
  }

  bool remove(int id) {
    final existing = _entriesById.remove(id);
    final node = _entryNode.remove(id);
    if (existing == null || node == null) return false;
    return node.entries.remove(existing);
  }

  List<OctreeEntry<T>> queryAabb(OctreeAabb region) {
    final out = <OctreeEntry<T>>[];
    void visit(_LooseNode<T> node) {
      if (!node.looseBounds.intersectsAabb(region)) return;
      for (final e in node.entries) {
        if (e.bounds.intersectsAabb(region)) out.add(e);
      }
      final ch = node.children;
      if (ch != null) {
        for (final c in ch) {
          visit(c);
        }
      }
    }

    visit(_root);
    return out;
  }

  List<OctreeEntry<T>> queryFrustum(FrustumCullSet frustum) {
    final out = <OctreeEntry<T>>[];
    void visit(_LooseNode<T> node) {
      if (!frustum.intersectsAabb(node.looseBounds)) return;
      for (final e in node.entries) {
        if (frustum.intersectsAabb(e.bounds)) out.add(e);
      }
      final ch = node.children;
      if (ch != null) {
        for (final c in ch) {
          visit(c);
        }
      }
    }

    visit(_root);
    return out;
  }

  /// Exposes internal node traversal for ray queries via [visitEntries].
  void traverseRayCandidateNodes(
    OctreeVec3 origin,
    OctreeVec3 dir,
    double maxDistance,
    void Function(List<OctreeEntry<T>> entries) visitor,
  ) {
    void visit(_LooseNode<T> node) {
      if (node.looseBounds.intersectRay(origin, dir, maxDist: maxDistance) == null) {
        return;
      }
      if (node.entries.isNotEmpty) {
        visitor(node.entries);
      }
      final ch = node.children;
      if (ch != null) {
        for (final c in ch) {
          visit(c);
        }
      }
    }

    visit(_root);
  }
}
