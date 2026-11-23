#!/usr/bin/env python3
"""Reindex the Roboflow classes and balance real and synthetic training images."""
import argparse
import hashlib
import random
import shutil
from pathlib import Path

import yaml

TARGETS=['bottle','hammer']


def examples(root,domain,split):
    if domain=='real':
        folder=root/('valid' if split=='val' else split)
        names=yaml.safe_load((root/'data.yaml').read_text())['names']
        entries=names.items() if isinstance(names,dict) else enumerate(names)
        mapping={int(i):TARGETS.index({'bottle':'bottle','Orange_hammer':'hammer','hammer':'hammer'}[name]) for i,name in entries}
        images,labels=folder/'images',folder/'labels'
    else:
        mapping={0:0,1:1};images,labels=root/'images'/split,root/'labels'/split
    for path in sorted(images.glob('*')):
        if path.suffix.lower() in {'.jpg','.jpeg','.png','.webp'}:
            filename=labels/(path.stem+'.txt')
            lines=filename.read_text().splitlines() if filename.exists() else []
            payload=[]
            for line in lines:
                if not line.strip():
                    continue
                label,*box=line.split()
                coordinates=list(map(float,box))
                if any(not 0.<=v<=1. for v in coordinates):
                    raise ValueError(f'bad YOLO annotations in {filename}')
                # Roboflow detection exports can include polygon annotations.
                # Train this detector on their enclosing image-space boxes.
                if len(coordinates)>=6 and len(coordinates)%2==0:
                    xs,ys=coordinates[::2],coordinates[1::2]
                    left,right,top,bottom=min(xs),max(xs),min(ys),max(ys)
                    coordinates=[(left+right)/2,(top+bottom)/2,right-left,bottom-top]
                if len(coordinates)!=4 or min(coordinates[2:])<=0:
                    raise ValueError(f'bad YOLO annotations in {filename}')
                box=[format(v,'.10g') for v in coordinates]
                payload.append(' '.join([str(mapping[int(label)]),*box]))
            yield path,payload


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('real','synthetic','output'): p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--seed',type=int,default=42)
    args=p.parse_args();rng=random.Random(args.seed);out=args.output.resolve();out.mkdir(parents=True,exist_ok=False)
    used=set()
    for split in ('test','val','train'):
        groups={k:list(examples(v,k,split)) for k,v in [('real',args.real),('synthetic',args.synthetic)]}
        if split=='train':
            prepared={}
            for domain,rows in groups.items():
                seen=set(used)
                unique=[]
                for path,data in rows:
                    digest=hashlib.sha256(path.read_bytes()).hexdigest()
                    if digest in seen:
                        continue
                    seen.add(digest); unique.append((path,data))
                prepared[domain]=unique
            length=min(map(len,prepared.values()))
            if not length: p.error('both domains must have distinct training images')
            groups={k:rng.sample(rows,length) for k,rows in prepared.items()}
        for domain,rows in groups.items():
            images=out/domain/'images'/split; labels=out/domain/'labels'/split
            images.mkdir(parents=True);labels.mkdir(parents=True)
            accepted=0
            for path,data in rows:
                digest=hashlib.sha256(path.read_bytes()).hexdigest()
                if digest in used:
                    continue
                used.add(digest);name=f'{domain}_{accepted:07d}';accepted+=1
                shutil.copyfile(path,images/(name+path.suffix.lower()))
                (labels/(name+'.txt')).write_text('\n'.join(data)+('\n' if data else ''))
            print(domain,split,accepted)
    common={'path':str(out),'names':TARGETS,'train':['real/images/train','synthetic/images/train']}
    for domain in ('mixed','real','synthetic'):
        sources=['real','synthetic'] if domain=='mixed' else [domain]
        data={**common,**{split:[f'{x}/images/{split}' for x in sources] for split in ('val','test')}}
        (out/(domain+'.yaml')).write_text(yaml.safe_dump(data,sort_keys=False))
    (out/'ATTRIBUTION.txt').write_text('Real images: hammer-bottle by urc, Roboflow Universe, CC BY 4.0.\nhttps://universe.roboflow.com/urc-cvc4i/hammer-bottle-l3zmd-odqnx\nOrange_hammer labels were remapped to hammer; polygons were converted to enclosing detection boxes.\nSynthetic model credits: see third_party/NOTICES.md from this project, including J-Toastie Hammer CC BY 3.0.\n')


if __name__=='__main__': main()
