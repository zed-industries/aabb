use std::cell::Cell;

struct AabbTree<T> {
    root: Option<usize>,
    nodes: Vec<Node<T>>,
}

impl<T> AabbTree<T> {
    pub fn insert_object(&mut self, aabb: Aabb, key: T) {
        let new_node = self.push_leaf(aabb, key);

        // If the tree is empty, make the root the new leaf.
        if self.root.is_none() {
            self.root = Some(new_node);
            return;
        }

        // Search for the best place to add the new leaf based on heuristics.
        let mut index = self.root.unwrap();
        while let Node::Internal {
            left, right, aabb, ..
        } = &self.nodes[index]
        {
            let area = aabb.get().merge(&self.nodes[new_node].aabb());

            let left_cost = area.merge(&self.nodes[*left].aabb()).half_perimeter();
            let right_cost = area.merge(&self.nodes[*right].aabb()).half_perimeter();

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
        let old_parent = self.nodes[sibling].parent();
        let new_parent = self.push_internal(old_parent, sibling, new_node);

        // If there was an old parent, we need to update its children indices.
        if let Some(old_parent) = old_parent {
            let Node::Internal { left, right, .. } = &mut self.nodes[old_parent] else {
                unreachable!();
            };

            if *left == sibling {
                *left = new_parent;
            } else {
                *right = new_parent;
            }
        } else {
            // If this was the root, the new parent is the new root.
            self.root = Some(new_parent);
        }

        // Walk up the tree fixing heights and areas.
        self.fix_abbs_upwards(new_parent);
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

    fn set_left(&mut self, new_left: usize) {
        if let Node::Internal { left, .. } = self {
            *left = new_left;
        } else {
            panic!("called set_left on a leaf node")
        }
    }

    fn set_right(&mut self, new_right: usize) {
        if let Node::Internal { right, .. } = self {
            *right = new_right;
        } else {
            panic!("called set_right on a leaf node")
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
struct Aabb {
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

    fn half_perimeter(&self) -> f32 {
        let width = self.max.x - self.min.x;
        let height = self.max.y - self.min.y;
        width + height
    }
}

#[derive(Default, Clone, Copy)]
struct Point {
    x: f32,
    y: f32,
}
