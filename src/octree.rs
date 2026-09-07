use std::ops::{Index, IndexMut};

use bevy::{
    math::{U8Vec3, UVec3},
    platform::collections::HashMap,
};

/// Persistent walkable state for a depth-first octree. Incl
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Debug)]
struct Cursor {
    index: u32,
    /// The length of the tree with the depth equal to `level` excluding
    /// any skipped levels.
    tree_len: u32,
}

impl Cursor {
    /// Returns the root cursor for an octree with the given depth,
    /// skipping the last `skipped` levels.
    const fn root<const DEPTH: u8>(skipped: u8) -> Self {
        Cursor {
            index: 0,
            tree_len: tree_len(DEPTH - skipped),
            level: DEPTH,
        }
    }

    const fn is_root<const DEPTH: u8>(self) -> bool {
        self.level == DEPTH
    }

    const fn is_valid<const DEPTH: u8>(self) -> bool {
        self.level <= DEPTH
    }

    const fn is_leaf(self) -> bool {
        self.level == 0
    }

    const fn _descend1(&mut self) {
        self.level -= 1;
        self.tree_len = (self.tree_len - 1) / 8;
    }

    const fn _descend2(&mut self, oct: u32) {
        self.index += 1 + oct * self.tree_len;
    }

    const fn descend(&mut self, oct: u32) {
        self._descend1();
        self._descend2(oct);
    }

    fn descend_with(&mut self, pos: UVec3) {
        self._descend1();
        let oct = octant(pos, self.level);
        self._descend2(oct);
    }

    const fn ascend(&mut self, oct: u32) {
        self.index -= 1 + oct * self.tree_len;
        self.level += 1;
        self.tree_len = (self.tree_len * 8) + 1;
    }

    fn ascend_with(&mut self, pos: UVec3) {
        let oct = octant(pos, self.level);
        self.ascend(oct);
    }

    fn ascended_with(mut self, pos: UVec3) -> Self {
        self.ascend_with(pos);
        self
    }

    fn descended_with(mut self, pos: UVec3) -> Self {
        self.descend_with(pos);
        self
    }
}

/// A complete tree storing a byte per node indexed by a [`Cursor`]
#[derive(Clone, Debug)]
struct CompleteByteTree {
    vec: Vec<u8>,
}

impl CompleteByteTree {
    fn new(depth: u8, fill: u8) -> Self {
        let len = tree_len(depth) as usize;
        let vec = vec![fill; len];
        Self { vec }
    }
}

impl Index<Cursor> for CompleteByteTree {
    type Output = u8;

    fn index(&self, cursor: Cursor) -> &Self::Output {
        &self.vec[cursor.index as usize]
    }
}

impl IndexMut<Cursor> for CompleteByteTree {
    fn index_mut(&mut self, cursor: Cursor) -> &mut Self::Output {
        &mut self.vec[cursor.index as usize]
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
struct Key {
    pos: U8Vec3,
    level: u8,
}

impl Key {
    fn new(pos: UVec3, level: u8) -> Self {
        let pos = (pos >> level).as_u8vec3();
        Self { level, pos }
    }
}

#[derive(Clone, Debug)]
struct OctreeInner<T> {
    values: HashMap<Key, T>,
    have_children: CompleteByteTree,
    inherit: CompleteByteTree,
}

impl<T> OctreeInner<T> {
    fn new(depth: u8) -> Self {
        Self {
            values: HashMap::new(),
            have_children: CompleteByteTree::new(h_depth(depth), 0x00),
            inherit: CompleteByteTree::new(i_depth(depth), 0xFF),
        }
    }
}

#[derive(Clone, Debug, Default)]
pub struct Octree<T, const D: u8 = 5> {
    root: T,
    inner: Option<OctreeInner<T>>,
}

impl<T, const D: u8> Octree<T, D>
where
    T: Clone + PartialEq,
{
    pub const fn new(root: T) -> Self {
        assert!(D > 0);
        assert!(D <= 8);
        Self { root, inner: None }
    }

    const fn root() -> Cursor {
        Cursor::root(D)
    }

    const fn h_root() -> Cursor {
        Cursor::root(h_depth(D))
    }

    const fn i_root() -> Cursor {
        Cursor::root(i_depth(D))
    }

    pub fn get(&self, pos: UVec3) -> &T {
        let Some(octree) = &self.inner else {
            return &self.root;
        };

        let cursor = Self::root();
        let h_cursor = Self::h_root();
        let i_cursor = Self::i_root();
        loop {
            cursor.descend_with(pos);
            let i_oct = octant(pos, cursor.level - 1);
            if get_child(&octree.have_children[i_cursor], i_oct) {

            }

            h_cursor.descended_with(pos);
            i_cursor.descended_with(pos);

        }
        let h_cursor = ChildrenHaveChildren::root(D)
        let cursor = octree.have_children.scan_towards_from_root(pos);
        let inheritee = octree.inherit.scan_away(cursor, pos);

        if inheritee.is_root() {
            self.root
        } else {
            *octree.values.get(&inheritee.key(pos)).unwrap()
        }

        // PSEUDO CODE
        //
        // walk inner.splits towards pos until we find a split node or hit level 0
        // now we know the position and the level to look at
        //
        // walk inner.inherits backwards from the level we ended at until we find a non-inherited node
        // if we hit the root return the root, it's NOT in the map because I expect many nodes to inherit the
        // root and don't want to be forced to hash a constant for a value that always exists
        //
        // this node must exist in the map so we unwrap it and return it
    }

    pub fn set(&mut self, pos: UVec3, value: T) {
        let inner = match &mut self.inner {
            Some(inner) => inner,
            None => {
                if value == self.root {
                    return;
                }
                self.inner.insert(OctreeInner::new())
            }
        };

        let cursor = inner.have_children.scan_towards_from_root(pos);
        let c_oct = cursor.octant(pos);
        let inheritee = inner.inherit.scan_away(cursor, pos);

        let old_value = if inheritee.is_root() {
            self.root
        } else {
            *inner.values.get(&inheritee.key(pos)).unwrap()
        };
        if old_value == value {
            return;
        }

        if cursor.level == 0 {
            let parent = cursor.away(pos);
            let parent_byte = inner.inherit.get_byte(parent);

            {
                let mut ancestor = parent;
                let mut cursor = cursor;
                let mut oct = c_oct;
                let mut ancestor_byte = parent_byte;
                let mut commit = false;

                loop {
                    if ancestor_byte != 1 << oct {
                        break;
                    }
                    commit = true;
                    cursor = ancestor;
                    ancestor = cursor.away(pos);
                    oct = cursor.octant(pos);
                    ancestor_byte = inner.inherit.get_byte(ancestor);

                    // reevaluate children
                    for oct in 0..8 {
                        let cursor = ancestor.next(oct);
                        let key = cursor.key(pos).with_oct(oct);

                        let other_value = *inner.values.get(&key).unwrap();
                        if value == other_value {
                            inner.values.remove(&key);
                            inner.inherit.set(cursor, true, oct);
                        }
                    }
                }

                if commit {
                    if cursor.is_root() {
                        self.root = value;
                    } else {
                        inner.values.insert(cursor.key(pos), value);
                    }
                    return;
                }
            }

            let doesnt_inherit = (parent_byte >> c_oct) & 1 == 0;
            if doesnt_inherit {
                let parent_inheritee = inner.inherit.scan_away(parent, pos);
                let parent_value = *inner.values.get(&parent_inheritee.key(pos)).unwrap();
                if parent_value == value {
                    inner.values.remove(&cursor.key(pos)).unwrap();

                    *inner.inherit.byte_mut(parent) |= 1 << c_oct;

                    let mut ancestor = parent;
                    let mut anc_byte = parent_byte;

                    loop {
                        if anc_byte != !0u8 {
                            break;
                        }

                        let mut any_have = false;
                        for oct in 0..8 {
                            let anc_child = ancestor.next(oct);
                            any_have |= inner.have_children.get(anc_child);
                        }
                        if any_have {
                            break;
                        }

                        if ancestor.is_root() {
                            self.inner = None;
                            return;
                        } else {
                            inner.have_children.set(ancestor, false);
                        }

                        ancestor = ancestor.away(pos);
                        anc_byte = inner.inherit.get_byte(ancestor);
                    }

                    return;
                }
            }

            inner.inherit.set(parent, false, c_oct);
            inner.values.insert(cursor.key(pos), value);
        } else {
            let mut last = cursor;
            let mut drill = cursor;
            while drill.level != 0 {
                inner.have_children.set(drill, true);
                last = drill;
                drill = drill.toward(pos);
            }
            inner.values.insert(cursor.key(pos), value);
            let oct = drill.octant(pos);
            inner.inherit.set(last, false, oct)
        }

        // FIRST if we don't have any inner AND the value is the same as the root, do nothing
        //
        // if we don't have any inner and the value is NOT the same as the root create a new inner with default values
        // then proceed as if we had an inner (because we do)
        //
        // at this point we have an inner:
        // walk our inner.splits towards pos until we find an unsplit node or reach level 0. (pause are these different? the only difference is that we might have to split an unsplit node)
        // if we reach level 0:
        //      walk inner.inherits backwards from our level until we find a non-inherited node or root
        //      compare our value to the read value, if they are the same, do nothing
        //      if they are different:
        //          go to the direct parent of our lvl 0 node, read it's child_inheritance
        //          if child_inheritance == 1 << our_oct {
        //              now we must set the map value at our parent to our value since we were the sole inheritor
        //
        //              ~~*map.insert(parent_key, value) \\ this may override another value OR it may not~~
        //              BUT WAIT... what if our parent is the sole inheritor too?
        //
        //              this must recurse until we see someone who isn't the sole inheritor and set their value
        //
        //              BUT DOULBE WAIT SINCE we just changed a non-leaf nodes value we need to check if any other
        //              values that we're once NOT able to inherit (because they were different) are not equal to their
        //              parent ALSO for performance reasons we should be doing THIS step after every recursion into checking
        //              if our parent is the sole inheritor as well (because at that point we're already sure that the parent in
        //              question will be equal to our value)
        //              You may be like "thats really fucking expensive" and I would be like:
        //              "yes... yes it is... but like super rare... so gotacha"
        //          }
        //          // WE Know we're not the sole inheritor BUT maybe we're not inheriting at all, if so we should check if our operaiton makes it possible to inherit
        //          I'm sorta confused see I wrote this as if we only cared about the direct parent of the node in question but most other
        //          operations recurse into the parents. Well... if we are inheriting from our direct parent then our value is either equal (early ret)
        //          sole inheritor (code above) or the old value is not equal to our direct parents value and we cannot change our direct parents value.
        //          Hence here we see if we need to store our value or if we can elide it.
        //          if child_inheritance >> our_oct & 1 == 0 {
        //              now we must check to see if our new edit can be simplified into an inherit
        //              we don't have to do this check if we are inheriting because we earlier checked the the current value and if we were inheriting it would
        //              be equal one
        //
        //              Since we already know that the value it's set to is NOT equal
        //              if map.get(parent_key) == value { // NOPE we have to recurse through the chain of inheritance not get our parent_key.
        //                  // you may be like "we already recursed through this exact chain" no we didn't the first time we recurse we know
        //                  // we didn't get anywhere past ourself because we just checked that we weren't inheriting. We're basically
        //                  // checking the chain above the chain earlier.
        //                  child_inheritance |= 1 << our_oct
        //                  map.remove(our_key) // we know this exists because an entry already exists (lvl0) and we are not inheriting
        //
        //                  but wait we must now check if the nodes are mergable
        //                  if child_inheritance == u8::MAX {
        //                      well fuck I guess they are: this means we must unsplit the parent. Thankfully we don't have to remove
        //                      any map values because they are guarnateed removed by the fact they were inherited
        //                      split[parent].set(false)
        //                  }
        //              }
        //          }
        //          finally we CANNOT inherit because we know that the inherited value cannot be set equal to our value
        //          else {
        //              // to get here we either know A: we weren't inheriting or B: we were inhering but cannot now inherit
        //              child_inheritance &= !(1 << our_oct);
        //              map.insert(our_key, value);
        //          }
        //
        // Alright say we didn't reach level 0 (shit this might get complicated):
        //      again we walk through the chain of inheritance and check if our current value is equal to the target and if so early return.
        //
        //      at this point we know that the value previously assigned here is NOT equal to the new one.
        //
        //      now here's a problem... we're trying to set a SINGLE cell and right now we're looking at some sorta cube of cells all of which
        //      are equally values as NOT our target value. In this case I think theres no chance of merging and our only option is to
        //      split all the way down the chain and insert a value at lvl0 equal to our target value.
        //
        //      thankfully we don't have to set any of the inheritance values because all inheritance values are true
        //      for merged cubes.
        //
        //      for level in level..0 {
        //          split[idx(pos, lvl)] = true;
        //      }
        //
        //      let bottom_lvl = 0
        //      inheritance[idx(pos, bottom_lvl)] = false;
        //
    }
}

/// Returns the number of nodes in the tree with the given depth.
///
/// Equal to \sum_{d=0}^{D} 8^{d} = \frac{8^{D+1}-1}{7}
#[inline]
const fn tree_len(depth: u8) -> u32 {
    let depth = depth as u32;
    (8u32.pow(depth + 1) - 1) / 7
}

/// Equivalent to [`subtree_len`]
const fn tree_len_no_leaves(depth: u32) -> u32 {
    subtree_len(depth)
}

/// Returns the number of nodes in the subtree with the given depth.
///
/// Equal to \sum_{d=0}^{D - 1} 8^{d} = \frac{8^D-1}{7}
const fn subtree_len(depth: u32) -> u32 {
    (8u32.pow(depth) - 1) / 7
}

#[inline]
fn octant(mut pos: UVec3, level: u8) -> u32 {
    pos >>= level;
    pos &= 1;
    pos.x | pos.y << 1 | pos.z << 2
}

#[inline]
fn with_octant(pos: UVec3, level: u32, octant: u32) -> UVec3 {
    let x = octant & 1;
    let y = (octant >> 1) & 1;
    let z = (octant >> 2) & 1;
    let set = UVec3::new(x, y, z) << level;
    let clear = UVec3::ONE << level;
    (pos & !clear) | set
}

fn get_child(byte: &u8, oct: u32) -> bool {
    (*byte >> oct) & 1 != 0
}

fn set_child(byte: &mut u8, oct: u32, value: bool) {
    if value {
        *byte |= 1 << oct;
    } else {
        *byte &= !(1 << oct);
    }
}

const fn h_depth(depth: u8) -> u8 {
    depth - 1
}

const fn i_depth(depth: u8) -> u8 {
    depth - 2
}
