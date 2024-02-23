use std::cell::Cell;

pub struct AabbTree<T> {
    root: Option<usize>,
    nodes: Vec<Node<T>>,
}

impl<T: Clone> AabbTree<T> {
    pub fn new() -> Self {
        AabbTree {
            root: None,
            nodes: Vec::new(),
        }
    }

    pub fn insert(&mut self, aabb: Aabb, key: T) -> Vec<T> {
        let new_node = self.push_leaf(aabb.clone(), key.clone());
        let mut intersections = Vec::new();

        // If the tree is empty, make the root the new leaf.
        if self.root.is_none() {
            self.root = Some(new_node);
            dbg!();
            return intersections;
        }

        // Search for the best place to add the new leaf based on heuristics.
        let mut index = self.root.unwrap();
        while let Node::Internal {
            left,
            right,
            aabb: node_aabb,
            ..
        } = &self.nodes[index]
        {
            let area = node_aabb.get().merge(&self.nodes[new_node].aabb());

            let left_cost = area.merge(&self.nodes[*left].aabb()).half_perimeter();
            let right_cost = area.merge(&self.nodes[*right].aabb()).half_perimeter();

            // If there is an intersection with either child, add their keys to the intersections vector.
            if area.intersects(&self.nodes[*left].aabb()) {
                self.collect_intersections(*left, &aabb, &mut intersections);
            }
            if area.intersects(&self.nodes[*right].aabb()) {
                self.collect_intersections(*right, &aabb, &mut intersections);
            }

            // Descend to the best-fit child, based on which one would increase
            // the surface area the least. This attempts to keep the tree balanced
            // in terms of surface area.
            if left_cost < right_cost {
                index = *left;
            } else {
                index = *right;
            }
        }

        // We've found a leaf ('index' now refers to a leaf node).
        // We'll insert a new parent node above the leaf and attach our new leaf to it.
        let sibling = index;

        // Check for collision with the located leaf node
        let Node::Leaf {
            aabb: leaf_aabb,
            data,
            ..
        } = &self.nodes[index]
        else {
            unreachable!();
        };
        if leaf_aabb.intersects(&aabb) {
            intersections.push(data.clone());
        }

        let old_parent = self.nodes[sibling].parent();
        let new_parent = self.push_internal(old_parent, sibling, new_node);

        // If there was an old parent, we need to update its children indices.
        if let Some(old_parent) = old_parent {
            let Node::Internal { left, right, .. } = &mut self.nodes[old_parent] else {
                unreachable!();
            };

            if *left == sibling {
                dbg!();
                *left = new_parent;
            } else {
                dbg!();
                *right = new_parent;
            }
        } else {
            dbg!();
            // If the old parent was the root, the new parent is the new root.
            self.root = Some(new_parent);
        }

        // Walk up the tree fixing heights and areas.
        self.fix_abbs_upwards(new_parent);

        intersections
    }

    fn collect_intersections(&self, index: usize, aabb: &Aabb, intersections: &mut Vec<T>) {
        match &self.nodes[index] {
            Node::Leaf {
                data,
                aabb: leaf_aabb,
                ..
            } => {
                if aabb.intersects(leaf_aabb) {
                    intersections.push(data.clone());
                }
            }
            Node::Internal { left, right, .. } => {
                self.collect_intersections(*left, aabb, intersections);
                self.collect_intersections(*right, aabb, intersections);
            }
        }
    }

    fn push_leaf(&mut self, aabb: Aabb, data: T) -> usize {
        self.nodes.push(Node::Leaf {
            parent: None,
            aabb,
            data,
        });
        self.nodes.len() - 1
    }

    fn push_internal(&mut self, parent: Option<usize>, left: usize, right: usize) -> usize {
        let new_aabb = self.nodes[left].aabb().merge(&self.nodes[right].aabb());
        self.nodes[left].set_parent(parent);
        self.nodes[right].set_parent(parent);
        self.nodes.push(Node::Internal {
            parent,
            aabb: Cell::new(new_aabb),
            left,
            right,
        });
        self.nodes.len() - 1
    }

    fn fix_abbs_upwards(&mut self, node: usize) {
        let mut node = Some(node);

        while let Some(index) = node {
            let Node::Internal {
                parent,
                left,
                right,
                aabb,
            } = &self.nodes[index]
            else {
                unreachable!();
            };

            // Update the AABB in the current node to include its children's AABBs.
            let left_aabb = self.nodes[*left].aabb().clone();
            let right_aabb = self.nodes[*right].aabb().clone();
            aabb.set(left_aabb.merge(&right_aabb));

            // Move up to the parent.
            node = *parent;
        }
    }
}

#[derive(Default, Clone, Copy)]
pub struct Aabb {
    min: Point,
    max: Point,
}

impl Aabb {
    fn merge(&self, other: &Aabb) -> Aabb {
        Aabb {
            min: Point {
                x: self.min.x.min(other.min.x),
                y: self.min.y.min(other.min.y),
            },
            max: Point {
                x: self.max.x.max(other.max.x),
                y: self.max.y.max(other.max.y),
            },
        }
    }

    fn intersects(&self, other: &Aabb) -> bool {
        dbg!(
            !(self.min.x > other.max.x
                || self.max.x < other.min.x
                || self.min.y > other.max.y
                || self.max.y < other.min.y)
        )
    }

    fn half_perimeter(&self) -> f32 {
        let width = self.max.x - self.min.x;
        let height = self.max.y - self.min.y;
        width + height
    }
}

enum Node<T> {
    Leaf {
        parent: Option<usize>,
        aabb: Aabb,
        data: T,
    },
    Internal {
        parent: Option<usize>,
        left: usize,
        right: usize,
        aabb: Cell<Aabb>,
    },
}

impl<T> Node<T> {
    fn parent(&self) -> Option<usize> {
        match self {
            Node::Leaf { parent, .. } | Node::Internal { parent, .. } => *parent,
        }
    }

    fn set_parent(&mut self, new_parent: Option<usize>) {
        match self {
            Node::Leaf { parent, .. } | Node::Internal { parent, .. } => {
                *parent = new_parent;
            }
        }
    }

    fn aabb(&self) -> Aabb {
        match self {
            Node::Leaf { aabb, .. } => aabb.clone(),
            Node::Internal { aabb, .. } => aabb.get(),
        }
    }
}

#[derive(Default, Clone, Copy)]
struct Point {
    x: f32,
    y: f32,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_aabb_insertion_with_two_aabbs() {
        let mut tree = AabbTree::new();
        let aabb1 = Aabb {
            min: Point { x: 0.0, y: 0.0 },
            max: Point { x: 10.0, y: 10.0 },
        };
        let aabb2 = Aabb {
            min: Point { x: 5.0, y: 5.0 },
            max: Point { x: 15.0, y: 15.0 },
        };

        // Insert the first AABB.
        let intersections = tree.insert(aabb1, "AABB1".to_string());
        assert!(
            intersections.is_empty(),
            "There should be no intersections after inserting the first AABB."
        );

        // Insert the second AABB, which overlaps with the first.
        let intersections = tree.insert(aabb2, "AABB2".to_string());
        assert_eq!(
            intersections,
            vec!["AABB1".to_string()],
            "There should be an intersection with the first AABB."
        );
    }
}
