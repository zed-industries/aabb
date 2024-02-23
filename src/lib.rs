use std::{fmt, iter};

#[derive(Debug)]
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

    pub fn insert(&mut self, new_aabb: Aabb, key: T, intersections: &mut Vec<T>) {
        let new_node = self.push_leaf(new_aabb, key.clone());

        // If the tree is empty, make the root the new leaf.
        if self.root.is_none() {
            self.root = Some(new_node);
            return;
        }

        // Search for the best place to add the new leaf based on heuristics.
        let mut old_parent = None;
        let mut index = self.root.unwrap();
        while let Node::Internal {
            left,
            right,
            aabb: node_aabb,
            ..
        } = &mut self.nodes[index]
        {
            let left = *left;
            let right = *right;
            *node_aabb = node_aabb.merge(new_aabb);
            old_parent = Some(index);

            // Descend to the best-fit child, based on which one would increase
            // the surface area the least. This attempts to keep the tree balanced
            // in terms of surface area. If there is an intersection with the other child,
            // add its keys to the intersections vector.
            let left_cost = new_aabb.merge(self.nodes[left].aabb()).half_perimeter();
            let right_cost = new_aabb.merge(self.nodes[right].aabb()).half_perimeter();
            if left_cost < right_cost {
                self.collect_intersections(right, new_aabb, intersections);
                index = left;
            } else {
                self.collect_intersections(left, new_aabb, intersections);
                index = right;
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
        if leaf_aabb.intersects(new_aabb) {
            intersections.push(data.clone());
        }

        let new_parent = self.push_internal(sibling, new_node);

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
            // If the old parent was the root, the new parent is the new root.
            self.root = Some(new_parent);
        }
    }

    pub fn iter(&self) -> impl Iterator<Item = (Aabb, &T)> {
        let mut stack = Vec::new();
        stack.extend(self.root);
        iter::from_fn(move || {
            while let Some(node_ix) = stack.pop() {
                match &self.nodes[node_ix] {
                    Node::Leaf { aabb, data, .. } => {
                        return Some((*aabb, data));
                    }
                    Node::Internal { left, right, .. } => {
                        stack.push(*left);
                        stack.push(*right);
                    }
                }
            }
            None
        })
    }

    fn collect_intersections(&self, index: usize, aabb: Aabb, intersections: &mut Vec<T>) {
        match &self.nodes[index] {
            Node::Leaf {
                data,
                aabb: node_aabb,
                ..
            } => {
                if aabb.intersects(*node_aabb) {
                    intersections.push(data.clone());
                }
            }
            Node::Internal {
                left,
                right,
                aabb: node_aabb,
                ..
            } => {
                if aabb.intersects(*node_aabb) {
                    self.collect_intersections(*left, aabb, intersections);
                    self.collect_intersections(*right, aabb, intersections);
                }
            }
        }
    }

    fn push_leaf(&mut self, aabb: Aabb, data: T) -> usize {
        self.nodes.push(Node::Leaf { aabb, data });
        self.nodes.len() - 1
    }

    fn push_internal(&mut self, left: usize, right: usize) -> usize {
        let new_aabb = self.nodes[left].aabb().merge(self.nodes[right].aabb());
        self.nodes.push(Node::Internal {
            aabb: new_aabb,
            left,
            right,
        });
        self.nodes.len() - 1
    }
}

#[derive(Default, Debug, Clone, Copy, PartialEq)]
pub struct Aabb {
    min: Point,
    max: Point,
}

impl Aabb {
    fn merge(self, other: Aabb) -> Aabb {
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

    fn intersects(self, other: Aabb) -> bool {
        !(self.min.x > other.max.x
            || self.max.x < other.min.x
            || self.min.y > other.max.y
            || self.max.y < other.min.y)
    }

    fn half_perimeter(self) -> f32 {
        let width = self.max.x - self.min.x;
        let height = self.max.y - self.min.y;
        width + height
    }
}

#[derive(Debug)]
enum Node<T> {
    Leaf {
        aabb: Aabb,
        data: T,
    },
    Internal {
        left: usize,
        right: usize,
        aabb: Aabb,
    },
}

impl<T> Node<T> {
    fn aabb(&self) -> Aabb {
        match self {
            Node::Leaf { aabb, .. } => *aabb,
            Node::Internal { aabb, .. } => *aabb,
        }
    }
}

#[derive(Default, Clone, Copy, PartialEq)]
struct Point {
    x: f32,
    y: f32,
}

impl fmt::Debug for Point {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "(x: {:.2}, y: {:.2})", self.x, self.y)
    }
}

#[cfg(test)]
mod tests {
    use std::{fs, path::Path};

    use super::*;
    use rand::{Rng, SeedableRng};

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
        let mut intersections = Vec::new();
        tree.insert(aabb1, "AABB1".to_string(), &mut intersections);
        assert!(
            intersections.is_empty(),
            "There should be no intersections after inserting the first AABB."
        );

        // Insert the second AABB, which overlaps with the first.
        let mut intersections = Vec::new();
        tree.insert(aabb2, "AABB2".to_string(), &mut intersections);
        assert_eq!(
            intersections,
            vec!["AABB1".to_string()],
            "There should be an intersection with the first AABB."
        );
    }

    #[test]
    fn test_random_iterations() {
        let max_aabbs = 10;

        let mut actual_intersections: Vec<usize> = Vec::new();
        for seed in 1..=10000 {
            // let seed = 1;
            let debug = false;
            if debug {
                let svg_path = Path::new("./svg");
                if svg_path.exists() {
                    fs::remove_dir_all("./svg").unwrap();
                }
                fs::create_dir_all("./svg").unwrap();
            }

            dbg!(seed);

            let mut tree = AabbTree::new();
            let mut rng = rand::rngs::StdRng::seed_from_u64(seed as u64);
            let mut expected_aabbs: Vec<(Aabb, usize)> = Vec::new();

            // Insert a random number of random AABBs into the tree.
            let num_aabbs = rng.gen_range(1..=max_aabbs);
            for key in 0..num_aabbs {
                let min_x: f32 = rng.gen_range(-100.0..100.0);
                let min_y: f32 = rng.gen_range(-100.0..100.0);
                let max_x: f32 = rng.gen_range(min_x..min_x + 50.0);
                let max_y: f32 = rng.gen_range(min_y..min_y + 50.0);
                let aabb = Aabb {
                    min: Point { x: min_x, y: min_y },
                    max: Point { x: max_x, y: max_y },
                };

                expected_aabbs.push((aabb, key));
                if debug {
                    println!("inserting {} with AABB: {:?}", key, aabb);
                    draw_aabbs(
                        format!("./svg/expected_aabbs_after_{}.svg", key),
                        &expected_aabbs,
                    );
                }

                // Insert the AABB into the tree and collect intersections.
                actual_intersections.clear();
                tree.insert(aabb, key, &mut actual_intersections);
                if debug {
                    draw_aabb_tree(format!("./svg/aabb_tree_after_{}.svg", key), &tree);
                }

                // Verify the tree contains all the AABBs.
                let mut actual_aabbs = tree
                    .iter()
                    .map(|(aabb, key)| (aabb, *key))
                    .collect::<Vec<_>>();
                actual_aabbs.sort_by_key(|(_, key)| *key);
                expected_aabbs.sort_by_key(|(_, key)| *key);
                assert_eq!(actual_aabbs, expected_aabbs);

                // Verify intersections by brute force comparison.
                let mut expected_intersections = expected_aabbs
                    .iter()
                    .filter(|(other_aabb, other_key)| {
                        aabb.intersects(*other_aabb) && *other_key != key
                    })
                    .map(|(_, other_key)| *other_key)
                    .collect::<Vec<_>>();
                expected_intersections.sort_unstable();
                actual_intersections.sort_unstable();
                assert_eq!(
                    actual_intersections, expected_intersections,
                    "The intersections returned by the tree do not match the expected intersections."
                );
            }
        }
    }

    fn draw_aabbs(svg_path: impl AsRef<Path>, aabbs: &[(Aabb, usize)]) {
        let mut svg_content = String::from(
            r#"<svg xmlns="http://www.w3.org/2000/svg" version="1.1" viewBox="-100 -100 200 200" style="border:1px solid black;">"#,
        );

        for (aabb, key) in aabbs {
            svg_content.push_str(&format!(
                r#"<rect x="{}" y="{}" width="{}" height="{}" style="fill:none;stroke:black;stroke-width:1" />"#,
                aabb.min.x,
                aabb.min.y,
                aabb.max.x - aabb.min.x,
                aabb.max.y - aabb.min.y
            ));
            svg_content.push_str(&format!(
                r#"<text x="{}" y="{}" font-size="3" text-anchor="middle" alignment-baseline="central">{}</text>"#,
                (aabb.min.x + aabb.max.x) / 2.0,
                (aabb.min.y + aabb.max.y) / 2.0,
                key
            ));
        }

        svg_content.push_str("</svg>");
        fs::write(svg_path, &svg_content).unwrap();
    }

    fn draw_aabb_tree(svg_path: impl AsRef<Path>, tree: &AabbTree<usize>) {
        let mut svg_content = String::from(
            r#"<svg xmlns="http://www.w3.org/2000/svg" version="1.1" viewBox="-100 -100 200 200" style="border:1px solid black;">"#,
        );

        fn draw_node<T: fmt::Debug>(svg_content: &mut String, nodes: &[Node<T>], index: usize) {
            match &nodes[index] {
                Node::Internal {
                    aabb, left, right, ..
                } => {
                    svg_content.push_str(&format!(
                        r#"<rect x="{}" y="{}" width="{}" height="{}" style="fill:rgba({},{},{},0.5);stroke:rgba({},{},{},1);stroke-width:1" />"#,
                        aabb.min.x,
                        aabb.min.y,
                        aabb.max.x - aabb.min.x,
                        aabb.max.y - aabb.min.y,
                        (index * 50) % 255, // Red component
                        (index * 120) % 255, // Green component
                        (index * 180) % 255, // Blue component
                        (index * 50) % 255, // Red stroke
                        (index * 120) % 255, // Green stroke
                        (index * 180) % 255  // Blue stroke
                    ));
                    draw_node(svg_content, nodes, *left);
                    draw_node(svg_content, nodes, *right);
                }
                Node::Leaf { aabb, data, .. } => {
                    svg_content.push_str(&format!(
                        r#"<rect x="{}" y="{}" width="{}" height="{}" style="fill:none;stroke:black;stroke-width:1" />"#,
                        aabb.min.x,
                        aabb.min.y,
                        aabb.max.x - aabb.min.x,
                        aabb.max.y - aabb.min.y
                    ));
                    svg_content.push_str(&format!(
                        r#"<text x="{}" y="{}" font-size="3" text-anchor="middle" alignment-baseline="central">{:?}</text>"#,
                        (aabb.min.x + aabb.max.x) / 2.0,
                        (aabb.min.y + aabb.max.y) / 2.0,
                        data
                    ));
                }
            }
        }

        if let Some(root) = tree.root {
            draw_node(&mut svg_content, &tree.nodes, root);
        }

        svg_content.push_str("</svg>");
        fs::write(svg_path, &svg_content).unwrap();
    }
}
