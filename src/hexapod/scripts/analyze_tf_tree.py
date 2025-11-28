#!/usr/bin/env python3
"""
TF Tree Analyzer - Shows complete transform structure
Finds if there's a world/odom frame we can use
"""

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
import time


class TFTreeAnalyzer(Node):
    def __init__(self):
        super().__init__('tf_tree_analyzer')
        
        self.transforms = {}  # Store all transforms
        self.frames = set()
        self.parent_map = {}  # child -> parent mapping
        
        self.create_subscription(TFMessage, '/tf', self.tf_callback, 10)
        self.create_subscription(TFMessage, '/tf_static', self.tf_static_callback, 10)
        
        self.timer = self.create_timer(3.0, self.analyze)
        self.start_time = time.time()
        
        print("\n" + "="*70)
        print("TF TREE ANALYZER - Finding world/odom frames")
        print("="*70)
        print("Collecting transforms...\n")
    
    def tf_callback(self, msg):
        for tf in msg.transforms:
            if tf.header.frame_id:  # Valid transform
                parent = tf.header.frame_id
                child = tf.child_frame_id
                
                # Store position
                pos = tf.transform.translation
                self.transforms[child] = {
                    'parent': parent,
                    'pos': (pos.x, pos.y, pos.z),
                    'type': 'dynamic'
                }
                
                self.frames.add(parent)
                self.frames.add(child)
                self.parent_map[child] = parent
    
    def tf_static_callback(self, msg):
        for tf in msg.transforms:
            if tf.header.frame_id:
                parent = tf.header.frame_id
                child = tf.child_frame_id
                
                pos = tf.transform.translation
                self.transforms[child] = {
                    'parent': parent,
                    'pos': (pos.x, pos.y, pos.z),
                    'type': 'static'
                }
                
                self.frames.add(parent)
                self.frames.add(child)
                self.parent_map[child] = parent
    
    def find_root_frames(self):
        """Find frames that are parents but not children (roots)"""
        children = set(self.parent_map.keys())
        parents = set(self.parent_map.values())
        roots = parents - children
        return roots
    
    def analyze(self):
        if not self.transforms:
            print("Still collecting transforms...")
            return
        
        elapsed = time.time() - self.start_time
        
        print("\n" + "="*70)
        print(f"TF TREE ANALYSIS (after {elapsed:.1f}s)")
        print("="*70)
        
        # Find root frames
        roots = self.find_root_frames()
        
        print(f"\n1. ROOT FRAMES (not children of anything): {len(roots)}")
        for root in sorted(roots):
            print(f"   • {root}")
        
        # Find base_link's parent
        base_link_info = None
        if 'base_link' in self.parent_map:
            base_link_info = {
                'parent': self.parent_map['base_link'],
                'data': self.transforms.get('base_link', {})
            }
        
        print(f"\n2. BASE_LINK STATUS:")
        if base_link_info:
            print(f"   Parent frame: {base_link_info['parent']}")
            print(f"   Type: {base_link_info['data'].get('type', 'unknown')}")
            print(f"   Position: {base_link_info['data'].get('pos', 'unknown')}")
        elif 'base_link' in roots:
            print(f"   base_link is a ROOT frame (not a child)")
            print(f"   This means it's the origin - can't track its movement!")
        else:
            print(f"   ❌ base_link not found in TF tree")
        
        # Check for common world/odom frames
        print(f"\n3. LOOKING FOR WORLD/ODOM FRAMES:")
        world_frames = ['world', 'odom', 'map', 'earth', 'gazebo']
        found_world = False
        for frame in world_frames:
            if frame in self.frames:
                print(f"   ✅ Found: {frame}")
                found_world = True
                
                # Check if base_link is child of this
                if frame == self.parent_map.get('base_link'):
                    print(f"      → base_link IS child of {frame}! ✅✅✅")
        
        if not found_world:
            print(f"   ❌ No world/odom frames found")
        
        # Show full tree structure
        print(f"\n4. COMPLETE FRAME TREE:")
        self.print_tree(roots)
        
        print("\n" + "="*70)
        print("SOLUTION:")
        print("="*70)
        
        if base_link_info and base_link_info['parent'] in ['world', 'odom', 'map']:
            print(f"✅ GOOD NEWS!")
            print(f"   base_link has parent: {base_link_info['parent']}")
            print(f"   You can track velocity using TF!")
            print(f"\n   Run: python3 velocity_tracker_fixed.py")
        elif 'base_link' in roots:
            print(f"❌ PROBLEM:")
            print(f"   base_link is the root frame (no parent)")
            print(f"   It's at origin (0,0,0) and doesn't move in TF")
            print(f"\n   SOLUTION: Need to enable odometry or pose publisher")
            print(f"   OR track movement using joint states + forward kinematics")
        else:
            print(f"⚠️  Unclear situation")
            print(f"   Share this output for custom solution")
        
        print("="*70 + "\n")
    
    def print_tree(self, roots, indent=0):
        """Print tree structure recursively"""
        # Print children of roots
        for root in sorted(roots):
            print("   " + "  " * indent + f"• {root}")
            
            # Find children
            children = [child for child, parent in self.parent_map.items() 
                       if parent == root]
            
            if children:
                for child in sorted(children):
                    tf_type = self.transforms[child].get('type', '?')
                    print("   " + "  " * (indent + 1) + 
                          f"→ {child} ({tf_type})")


def main():
    rclpy.init()
    node = TFTreeAnalyzer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nAnalysis stopped\n")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
