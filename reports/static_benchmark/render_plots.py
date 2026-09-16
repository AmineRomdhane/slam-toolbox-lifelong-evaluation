"""Regenerate report figures from included frozen JSON snapshots only.
Requires NumPy and Matplotlib; does not use ROS or original experiment outputs.
"""
from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
OUT=Path(__file__).resolve().parent
COHORTS=['r1','r2','r3']
poses={r:json.loads((OUT/'data'/f'{r}_pose_system.json').read_text()) for r in COHORTS}
maps={r:json.loads((OUT/'data'/f'{r}_map.json').read_text()) for r in COHORTS}
plt.rcParams.update({'font.family':'DejaVu Sans','font.size':10,'axes.spines.top':False,'axes.spines.right':False,'svg.fonttype':'none'})
COLORS=['#0072B2','#D55E00','#009E73']
def plot(name,items,title):
 fig,axes=plt.subplots(2,2,figsize=(10,6.5));fig.subplots_adjust(top=.87,bottom=.11,wspace=.32,hspace=.45)
 for ax,(kind,key,label,unit,scale) in zip(axes.flat,items):
  values=[]
  for r in COHORTS:
   report=poses[r] if kind=='pose' else maps[r];vals=report['metrics'][key]['individual_values'];values.append(np.array([x['value'] if isinstance(x,dict) else x for x in vals])*scale)
  bp=ax.boxplot(values,positions=[1,2,3],widths=.43,patch_artist=True,showfliers=False,whis=1.5)
  for box,c in zip(bp['boxes'],COLORS):box.set(facecolor=c,alpha=.16,edgecolor=c)
  for med in bp['medians']:med.set(color='#24292f',linewidth=1.4)
  for i,(v,c) in enumerate(zip(values,COLORS),1):ax.scatter(i+np.linspace(-.13,.13,len(v)),v,s=25,color=c,alpha=.85,zorder=3,edgecolors='white',linewidth=.4)
  lo=min(v.min() for v in values);hi=max(v.max() for v in values);pad=max((hi-lo)*.18,abs(hi)*.008,1e-6);ax.set_ylim(max(0,lo-pad),hi+pad)
  ax.set_xticks([1,2,3],['R1','R2','R3 valid']);ax.set_title(label,loc='left',fontweight='bold');ax.set_ylabel(unit);ax.grid(axis='y',alpha=.18);ax.set_axisbelow(True);ax.ticklabel_format(axis='y',style='plain',useOffset=False)
 fig.suptitle(title,fontweight='bold',fontsize=15);fig.text(.5,.025,'10 dots per route · box: median and IQR · whiskers: 1.5×IQR · zoomed y-axes',ha='center',fontsize=9,color='#57606a')
 for ext in ['png','svg']:fig.savefig(OUT/'assets'/f'{name}.{ext}',dpi=180,facecolor='white')
 plt.close(fig)
plot('pose_distributions',[('pose','ate_translation_rmse','Translational ATE RMSE ↓','mm',1000),('pose','ape_yaw_rmse','Yaw APE RMSE ↓','mrad',1000),('pose','rpe_1m_translation_rmse','1 m translational RPE RMSE ↓','mm',1000),('pose','rpe_1m_rotation_rmse','1 m rotational RPE RMSE ↓','mrad',1000)],'Pose accuracy across valid static trials')
plot('system_distributions',[('pose','active_cpu_mean','Active CPU mean','% of one logical CPU',1),('pose','active_cpu_peak','Active CPU peak','% of one logical CPU',1),('pose','active_rss_mean','Active RSS mean','MiB',1),('pose','active_rss_peak','Active RSS peak','MiB',1)],'SLAM-process resource distributions')
plot('map_distributions',[('map','occupied_precision','Occupied precision ↑','fraction',1),('map','occupied_recall','Occupied recall ↑','fraction',1),('map','occupied_f1','Occupied F1 ↑','fraction',1),('map','boundary_distance_mean_m','Symmetric boundary mean ↓','mm',1000)],'Map quality in each reference observed domain')
