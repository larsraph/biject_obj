use std::ops::{Index, IndexMut};

use bevy::{
    math::{U8Vec3, UVec3},
    platform::collections::HashMap,
};

#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Debug)]
struct Cursor {
    index: u32,
    tree_len: u32,
}

impl Cursor {
    const fn root(depth: u32) -> Self {
        Cursor {
            index: 0,
            tree_len: tree_len(depth),
        }
    }

    const fn descend(&mut self, octant: u32) {
        self.tree_len = (self.tree_len - 1) / 8;
        self.index += 1 + octant * self.tree_len;
    }

    const fn ascend(&mut self, octant: u32) {
        self.index -= 1 + octant * self.tree_len;
        self.tree_len = (self.tree_len * 8) + 1;
    }

    const fn ascended(mut self, octant: u32) -> Self {
        self.ascend(octant);
        self
    }

    const fn descended(mut self, octant: u32) -> Self {
        self.descend(octant);
        self
    }
}

#[derive(Clone, Debug)]
struct CompleteByteOctree {
    vec: Vec<u8>,
}

impl CompleteByteOctree {
    fn new(depth: u32, fill: u8) -> Self {
        let len = tree_len(depth) as usize;
        let vec = vec![fill; len];
        Self { vec }
    }

    fn get_child(&self, cursor: Cursor, octant: u32) -> bool {
        get_child(&self[cursor], octant)
    }

    fn set_child(&mut self, cursor: Cursor, octant: u32, value: bool) {
        set_child(&mut self[cursor], octant, value);
    }
}

impl Index<Cursor> for CompleteByteOctree {
    type Output = u8;

    fn index(&self, cursor: Cursor) -> &Self::Output {
        &self.vec[cursor.index as usize]
    }
}

impl IndexMut<Cursor> for CompleteByteOctree {
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
    fn new(pos: UVec3, level: u32) -> Self {
        let level = level as u8;
        let pos = (pos >> level).as_u8vec3();
        Self { level, pos }
    }

    fn with_octant(mut self, octant: u32) -> Self {
        let x = octant & 1;
        let y = (octant >> 1) & 1;
        let z = (octant >> 2) & 1;
        self.pos += UVec3::new(x, y, z).as_u8vec3();
        self
    }
}

#[derive(Clone, Debug)]
struct OctreeInner<T> {
    values: HashMap<Key, T>,
    children_have_children: CompleteByteOctree,
    children_inherit_this: CompleteByteOctree,
}

impl<T> OctreeInner<T> {
    fn new(depth: u32) -> Self {
        Self {
            values: HashMap::new(),
            children_have_children: CompleteByteOctree::new(depth - 1, 0x00),
            children_inherit_this: CompleteByteOctree::new(depth - 2, 0xFF),
        }
    }
}

#[derive(Clone, Debug, Default)]
pub struct Octree<T, const D: u32 = 5> {
    root: T,
    inner: Option<OctreeInner<T>>,
}

impl<T, const D: u32> Octree<T, D>
where
    T: Clone + PartialEq,
{
    pub const fn new(root: T) -> Self {
        // OctreeInner panics for D < 2.
        // Technically D should be able to equal 0 and 1 but I'm too lazy to handle it.
        assert!(D > 1);
        assert!(D <= 8);
        Self { root, inner: None }
    }

    const fn h_root() -> Cursor {
        Cursor::root(D - 1)
    }

    const fn i_root() -> Cursor {
        Cursor::root(D - 2)
    }

    pub fn _get(&self, pos: UVec3) -> &T {
        let Some(OctreeInner {
            values,
            children_have_children: have,
            children_inherit_this: inherit,
        }) = &self.inner
        else {
            return &self.root;
        };

        let mut h_cursor = Self::h_root();
        let mut i_cursor = Self::i_root();

        let mut level = D - 1;
        let mut octant = get_octant(pos, level);

        while level != 0 {
            if !have.get_child(h_cursor, octant) {
                break;
            }

            h_cursor.descend(octant);
            i_cursor.descend(octant);
            level -= 1;
            octant = get_octant(pos, level);
        }

        while level != D {
            if !inherit.get_child(i_cursor, octant) {
                return values.get(&Key::new(pos, level)).unwrap();
            }

            i_cursor.ascend(octant);
            level += 1;
            octant = get_octant(pos, level);
        }
        &self.root

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
        let OctreeInner {
            values,
            children_have_children: have,
            children_inherit_this: inherit,
        } = match &mut self.inner {
            Some(inner) => inner,
            None => {
                if value == self.root {
                    return;
                }
                self.inner.insert(OctreeInner::new(D))
            }
        };

        // Keep in mind these cursors always point to the parent of the node in question.
        let mut h_cursor = Self::h_root();
        let mut i_cursor = Self::i_root();

        let mut level = D - 1;
        let mut octant = get_octant(pos, level);

        while level != 0 {
            if !have.get_child(h_cursor, octant) {
                break;
            }

            h_cursor.descend(octant);
            i_cursor.descend(octant);
            level -= 1;
            octant = get_octant(pos, level);
        }
        let h_cursor = h_cursor;
        let i_cursor = i_cursor;
        let octant = octant;
        let level = level;

        let mut i_cursor2 = i_cursor;
        let mut octant2 = octant;
        let mut level2 = level;
        let i_byte = inherit[i_cursor];

        let mut old_value = &self.root;
        if !get_child(&i_byte, octant2) {
            i_cursor2.ascend(octant2);
            level2 += 1;
            octant2 = get_octant(pos, level2);

            while level2 != D {
                if !inherit.get_child(i_cursor2, octant2) {
                    old_value = values.get(&Key::new(pos, level2)).unwrap();
                    break;
                }

                i_cursor2.ascend(octant2);
                level2 += 1;
                octant2 = get_octant(pos, level2);
            }
        }
        if *old_value == value {
            return;
        }

        if level == 0 {
            {
                let mut i_cursor2 = i_cursor;
                let mut octant2 = octant;
                let mut level2 = level;
                let mut i_byte2 = i_byte;

                let mut once = false;

                loop {
                    if i_byte2 != 1 << octant2 {
                        break;
                    }

                    for octant in 0..8 {
                        if octant == octant2 {
                            continue;
                        }

                        let key = Key::new(pos, level2).with_octant(octant);

                        let other_value = values.get(&key).unwrap();
                        if *other_value == value {
                            values.remove(&key);
                            inherit.set_child(i_cursor2, octant, true);
                        }
                    }

                    if level2 == D {
                        self.root = value;
                        return;
                    }

                    i_cursor2.ascend(octant2);
                    i_byte2 = inherit[i_cursor2];
                    level2 += 1;
                    octant2 = get_octant(pos, level2);

                    once = true;
                }

                if once {
                    values.insert(Key::new(pos, level), value);
                    return;
                }
            }

            if !get_child(&i_byte, octant) {
                let mut i_cursor2 = i_cursor.ascended(octant);
                let mut level2 = level + 1;
                let mut octant2 = get_octant(pos, level2);

                let mut parent_value = &self.root;
                while level2 != D {
                    if !inherit.get_child(i_cursor2, octant2) {
                        parent_value = values.get(&Key::new(pos, level2)).unwrap();
                        break;
                    }

                    i_cursor2.ascend(octant2);
                    level2 += 1;
                    octant2 = get_octant(pos, level2);
                }

                if *parent_value == value {
                    values.remove(&Key::new(pos, level)).unwrap();

                    let i_byte = &mut inherit[i_cursor];
                    set_child(i_byte, octant, true);

                    if *i_byte == 0xFF {
                        let mut h_cursor2 = h_cursor.ascended(octant);
                        let mut i_cursor2 = i_cursor.ascended(octant);
                        let mut level2 = level + 1;
                        let mut octant2 = get_octant(pos, level2);

                        let h_byte2 = &mut have[h_cursor2];
                        set_child(h_byte2, octant2, false);
                        let mut h_byte2 = *h_byte2;

                        loop {
                            if h_byte2 != 0x00 && inherit[i_cursor2] != 0xFF {
                                break;
                            }

                            level2 += 1;
                            if level2 == D {
                                self.inner = None;
                                return;
                            }
                            h_cursor2.ascend(octant2);
                            i_cursor2.ascend(octant2);
                            octant2 = get_octant(pos, level2);

                            let byte = &mut have[h_cursor2];
                            set_child(byte, octant2, false);
                            h_byte2 = *byte;
                        }
                    }

                    return;
                } else {
                    values.insert(Key::new(pos, level), value);
                }
            } else {
                inherit.set_child(i_cursor, octant, false);
                values.insert(Key::new(pos, level), value);
            }
        } else {
            let mut i_cursor2 = i_cursor;
            let mut level2 = level;
            let mut octant2 = octant;

            while level2 != 0 {
                have.set_child(i_cursor2, octant2, true);

                i_cursor2.descend(octant2);
                level2 -= 1;
                octant2 = get_octant(pos, level2);
            }
            values.insert(Key::new(pos, level), value);
            inherit.set_child(i_cursor2, octant2, false);
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
const fn tree_len(depth: u32) -> u32 {
    (8u32.pow(depth + 1) - 1) / 7
}

fn get_octant(mut pos: UVec3, level: u32) -> u32 {
    pos >>= level;
    pos &= 1;
    pos.x | pos.y << 1 | pos.z << 2
}

const fn get_child(byte: &u8, octant: u32) -> bool {
    (*byte >> octant) & 1 != 0
}

const fn set_child(byte: &mut u8, octant: u32, value: bool) {
    if value {
        *byte |= 1 << octant;
    } else {
        *byte &= !(1 << octant);
    }
}
