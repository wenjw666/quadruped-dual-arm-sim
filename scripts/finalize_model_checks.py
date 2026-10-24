from pathlib import Path
p=Path('/ws/scripts/render_refined_robot.py');s=p.read_text(encoding='utf-8-sig').replace("('upper_detail',155,-12,1.45,[.10,0,.98])]","('upper_detail',155,-12,1.45,[.10,0,.98]),('gripper_detail',145,-12,.62,[.25,.47,.72])]")
p.write_text(s)
p=Path('/ws/scripts/test_refined_robot.py');s=p.read_text(encoding='utf-8-sig').replace('[[.64,.17,.01],[.64,.17,.13],[.60,.17,.02]]','[[.64,.32,.01],[.64,.32,.13],[.60,.32,.02]]');p.write_text(s)
p=Path('/ws/scripts/check_refined_collisions.py');s=p.read_text(encoding='utf-8-sig').replace('[.64,.17,.02]','[.64,.32,.02]');p.write_text(s)
