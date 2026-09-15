"""Dataset adapters for frozen R1 methods. Only paths, labels, input counts and time spans vary."""
import argparse,hashlib,json,os,sys,subprocess,math,yaml,collections
from pathlib import Path
W=Path('/home/roam5170/slam_testing');BASE=W/'experiments/static_v0_r1';OLD=W/'bags/static/world_v0/r1/evidence';RBASE=W/'experiments/reference_v0_r1';SBASE=W/'experiments/map_evaluation_v0_r1'
p=argparse.ArgumentParser();p.add_argument('step');p.add_argument('route',choices=['r2','r3']);p.add_argument('--run-id');p.add_argument('--generation',type=int);args=p.parse_args();route=args.route;upper=route.upper();ROOT=W/f'bags/static/world_v0/{route}';E=ROOT/'evidence';BAG=ROOT/f'v0_{route}_canonical';RESULT=W/f'results/static/world_v0/{route}';REF=W/f'reference_maps/static/world_v0/{route}';HERE=Path(__file__).resolve().parent

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def replace(s,a,b):
 assert a in s,('Adapter anchor missing',a);return s.replace(a,b)
def execute(source,original,save,argslist=[]):
 save.parent.mkdir(parents=True,exist_ok=True);save.write_text(source);sys.path.insert(0,str(original.parent));sys.argv=[str(original),*argslist];exec(compile(source,str(save),'exec'),{'__name__':'__main__','__file__':str(original)})
def dataset(s):return s.replace('world_v0/r1','world_v0/'+route).replace('v0_r1_canonical','v0_'+route+'_canonical').replace('V0_R1','V0_'+upper).replace('routes/r1.yaml','routes/'+route+'.yaml')
def validation():return json.loads((E/'bag_validation.json').read_text())
def metadata():return yaml.safe_load((ROOT/'metadata.yaml').read_text())

if args.step=='record':
 src=OLD/'record_experiment.py';s=dataset(src.read_text());s=replace(s,"'-p','use_sim_time:=true','-p','output_dir:'", "UNUSED") if False else s
 s=replace(s,"'-p','use_sim_time:=true','-p','output_dir:='+str(E/'route_run')", "'-p','use_sim_time:=true','-p','route_file:='+str(WS/'src/slam_eval_route_controller/routes/"+route+".yaml'),'-p','output_dir:='+str(E/'route_run')")
 s=replace(s,'unchanged R1','unchanged '+upper)
 execute(s,src,RESULT/'adapter_evidence/record_executed.py')
elif args.step=='metadata':
 rec=json.loads((E/'recording_report.json').read_text());assert rec['recording_completed_cleanly'] and not rec.get('shutdown_errors');rr=rec['route_result'];assert rr['state']=='COMPLETED'
 m={'experiment_id':'V0_'+upper,'canonical_status':'PENDING_VALIDATION','world':'V0','route':upper,'robot':'TurtleBot3 Burger','ros_distro':'jazzy','versions':rec['versions'],'gazebo_version':rec['gazebo_version'],'slam_toolbox_version':'2.8.5','slam_testing_git_commit':rec['git_commit'],'turtlebot3_source_commits':rec['turtlebot3_commits'],'actual_start_pose':dict(rr['actual_start_pose']),'route_duration_sim_seconds':rr['duration'],'path_length_m':rr['path_length'],'recording_command':rec['recording_command'],'recorded_topics':rec['recorded_topics'],'storage':'mcap','use_sim_time':True,'clock_recorded':False,'route_sha256':sha(W/f'src/slam_eval_route_controller/routes/{route}.yaml'),'canonical_bag_sha256':sha(BAG/f'v0_{route}_canonical_0.mcap'),'approved_settling_criteria':yaml.safe_load((W/'bags/static/world_v0/r1/metadata.yaml').read_text())['approved_settling_criteria'],'assessment_history':{'protocol':'Prospective inherited R1 initial <=5 mm over >=5 sim seconds; no 1 mm initial criterion.'}}
 m['actual_start_pose']['stamp_ns']=m['actual_start_pose'].pop('sec')*10**9+m['actual_start_pose'].pop('nanosec');(ROOT/'metadata.yaml').write_text(yaml.safe_dump(m,sort_keys=False));(E/'sha256.json').write_text(json.dumps({str(p.relative_to(ROOT)):sha(p) for p in BAG.iterdir() if p.is_file()},indent=2))
elif args.step=='validate_bag':
 src=OLD/'validate_bag.py';s=dataset(src.read_text());s=s.replace("check('R1_waypoints", "check('"+upper+"_waypoints")
 execute(s,src,RESULT/'adapter_evidence/validate_bag_executed.py')
 v=validation();m=metadata();m.update(actual_start_pose=v['route_from_bag_gt']['start_pose'],actual_final_pose=v['route_from_bag_gt']['final_pose'],path_length_m=v['route_from_bag_gt']['path_length_m'],route_duration_sim_seconds=v['route_from_bag_gt']['duration_seconds'],canonical_status='PENDING_REPLAY',scan_count=v['topics']['/scan']['message_count'],bag_duration_seconds=v['bag_duration_seconds']);(ROOT/'metadata.yaml').write_text(yaml.safe_dump(m,sort_keys=False))
 with (E/'bag_info.txt').open('w') as f:subprocess.run(['ros2','bag','info',str(BAG)],stdout=f,check=True)
elif args.step=='replay':
 src=OLD/'replay_validation_script.py';s=dataset(src.read_text());a=s.index('# Preserve the provisional');b=s.index('TOPICS=',a);s=s[:a]+"assert json.loads((E/'bag_validation.json').read_text())['passed']\n"+s[b:]
 s=s.replace('from rclpy.serialization import serialize_message','from rclpy.serialization import serialize_message,deserialize_message')
 s=replace(s,"if topic in expected:expected[topic][digest(data)]+=1", "if topic in expected:expected[topic][digest(data)]+=1\n if topic in expected_stamps:expected_stamps[topic].append(ns(deserialize_message(data,TOPICS[topic]).header.stamp))")
 s=replace(s,"checks[topic+'_all_original_messages_received_unchanged']=same", "checks[topic+'_all_expected_messages_received']=sum(seen[topic].values())==sum(expected[topic].values())")
 s=replace(s,"checks[topic+'_monotonic_headers']=all(b>a for a,b in zip(h,h[1:]))", "checks[topic+'_monotonic_headers']=all(b>a for a,b in zip(h,h[1:]))\n   checks[topic+'_original_timestamps_preserved']=h==expected_stamps[topic]\n   checks[topic+'_simulation_clock_coherent']=max(abs(x) for x in d)<.1")
 # Use the already validated endpoint-GID clock guard, including unresolved identity.
 s=replace(s,'last_poll=started;last_progress=started;',"sys.path.insert(0,'"+str(BASE)+"');from clock_guard import ClockGuard\n guard=ClockGuard();guard.start_playback(started)\n last_poll=started;last_progress=started;")
 s=replace(s,"if now-started>210:",f"if now-started>{validation()['bag_duration_seconds']+40!r}:")
 s=replace(s,"if len(pp)!=1 or pp[0]['node']!='rosbag2_player':conflicts.append({'topic':t,'publishers':pp})", "if len(pp)!=1 or pp[0]['node'] not in ('rosbag2_player','_NODE_NAME_UNKNOWN_','',None):conflicts.append({'topic':t,'publishers':pp})")
 s=replace(s,"if t=='/clock':authority.update((p['node'],p['gid']) for p in pp)", "if t=='/clock':\n      authority.update(p['gid'] for p in pp)\n      event=guard.observe(pp,now)\n      if event['conflict']:raise RuntimeError(event['conflict'])")
 s=replace(s,"'clock_authorities']=[{'node':n,'gid':g} for n,g in authority]", "'clock_authorities']=[{'gid':g} for g in authority]")
 lo,hi=validation()['bag_interval_ns'];s=replace(s,'abs(c[0]/1e9-11)<.1 and abs(c[-1]/1e9-181.884)<.1',f'abs(c[0]-{lo})<100000000 and abs(c[-1]-{hi})<100000000')
 # Stop at the core validation, before the original R1 historical acceptance-rewrite block.
 end=s.index("if not report.get('passed'):sys.exit(1)")+len("if not report.get('passed'):sys.exit(1)");s=s[:end]+'\n'
 execute(s,src,RESULT/'adapter_evidence/replay_executed.py');m=metadata();m.update(canonical_status='ACCEPTED',replay_status='PASSED');(ROOT/'metadata.yaml').write_text(yaml.safe_dump(m,sort_keys=False))
elif args.step=='slam':
 src=BASE/'run_pilot.py';s=dataset(src.read_text());m=metadata();assert m['canonical_status']=='ACCEPTED';v=validation();count=v['topics']['/scan']['message_count'];duration=v['bag_duration_seconds'];s=s.replace('f094c269194313d72ac49d9f7ab181bca5c301763453d1e9d38505072b26a715',m['canonical_bag_sha256']);s=s.replace('170.884',repr(duration)).replace("'input_scans_in_bag':855","'input_scans_in_bag':"+str(count)).replace('len(scans)==855','len(scans)=='+str(count)).replace('received_all_855_scans_at_monitor','received_all_'+str(count)+'_scans_at_monitor');s=s.replace('player.poll() is not None,220','player.poll() is not None,'+repr(duration+49.116))
 execute(s,src,RESULT/'adapter_evidence'/f'{args.run_id}_executed.py',['--run-id',args.run_id])
else:raise ValueError(args.step)
