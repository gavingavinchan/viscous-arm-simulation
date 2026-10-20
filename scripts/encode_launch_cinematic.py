"""Verify complete numbered frames, encode masters/web, and package review page."""
from pathlib import Path
import argparse,json,subprocess,shutil,wave
import numpy as np
import imageio_ffmpeg
from PIL import Image,ImageDraw,ImageFont
ROOT=Path(__file__).resolve().parents[1];BASE=ROOT/'cinematic/launch'
ff=imageio_ffmpeg.get_ffmpeg_exe()
def run(cmd):subprocess.run([ff,'-hide_banner','-loglevel','error','-y',*cmd],check=True)
def score():
    path=BASE/'launch_soundtrack.wav'
    rate=48000;t=np.arange(32*rate)/rate;rng=np.random.default_rng(2026)
    # Original sparse electronic bed: restrained fifths, soft pulses and cut swells.
    bed=np.zeros_like(t)
    for hz,gain in [(55,.035),(110,.055),(164.8138,.05),(220,.04),(329.6276,.035),(440,.018),(659.255,.009)]:bed+=gain*np.sin(2*np.pi*hz*t+.3*np.sin(2*np.pi*.08*t))
    bed*=.55+.12*np.sin(2*np.pi*.12*t)
    for cut in [4,8,11,14,17,20,23,26]:
        dt=t-cut;env=np.exp(-np.maximum(dt,0)*6)*(dt>=0);bed+=.035*np.sin(2*np.pi*(65*dt+12*(1-np.exp(-np.maximum(dt,0)*12))))*env
        env=np.exp(-((t-(cut-.15))/.35)**2);noise=rng.normal(0,1,len(t));smooth=np.convolve(noise,np.ones(30)/30,'same');bed+=.035*smooth*env
    fade=np.minimum(t/1.7,1)*np.minimum(np.maximum(32-t,0)/2,1);bed*=fade
    bed*=1.65
    right=.985*bed+.005*np.sin(2*np.pi*220.2*t)*fade
    pcm=(np.stack([bed,right],axis=1)*32767).astype('<i2')
    with wave.open(str(path),'wb') as w:w.setnchannels(2);w.setsampwidth(2);w.setframerate(rate);w.writeframes(pcm.tobytes())
    return path

def encode(finish):
    out=BASE/finish;m=json.loads((out/'motion.json').read_text());expected=[out/'frames'/f'frame_{f:04d}.png' for f in range(1,m['end']+1)]
    for path in expected:
        if not path.exists():raise RuntimeError(f'Missing {path}')
        with Image.open(path) as im:
            if im.size!=(1920,1080):raise RuntimeError(f'Wrong dimensions {path}: {im.size}')
            im.verify()
    source=['-framerate','24','-start_number','1','-i',str(out/'frames/frame_%04d.png'),'-i',str(score())]
    color=['-pix_fmt','yuv420p','-color_primaries','bt709','-color_trc','bt709','-colorspace','bt709']
    # Short fades in/out; cuts otherwise stay deliberate and clean.
    grade='fade=t=in:st=0:d=0.5,fade=t=out:st=31.25:d=0.75'
    outputs={}
    for suffix,scale,crf in [('',None,'17'),('_web','1280:720','23')]:
        path=out/f'viscous_launch{suffix}.mp4';vf=grade+',scale='+(scale or '1920:1080')+':out_color_matrix=bt709'
        run([*source,'-vf',vf,'-c:v','libx264','-preset','slow','-crf',crf,*color,'-c:a','aac','-b:a','192k','-shortest','-movflags','+faststart',str(path)])
        run(['-i',str(path),'-f','null','-'])
        reader=imageio_ffmpeg.read_frames(str(path));meta=next(reader);reader.close();assert meta['fps']==24 and abs(meta['duration']-32)<.1
        outputs[path.name]={'size':meta['size'],'fps':meta['fps'],'duration':meta['duration'],'bytes':path.stat().st_size,'full_decode_passed':True}
    with Image.open(out/'frames/frame_0720.png') as im:im.save(out/'poster.jpg',quality=96)
    (out/'review').mkdir(exist_ok=True)
    for shot in m['shots']:shutil.copyfile(out/'frames'/f"frame_{shot['review']:04d}.png",out/'review'/f"{shot['id']}.png")
    contact(out,m)
    (out/'render_report.json').write_text(json.dumps({'render_settings':json.loads((out/'frames/settings.json').read_text()),'render_device':m['device'],'rendered_frames':len(expected),'frame_dimensions':[1920,1080],'outputs':outputs,'sound':'Original procedural stereo score; no borrowed music.','source':m['source_urdf']},indent=2)+'\n')
    print('ENCODED',finish,flush=True)

def contact(out,m):
    font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',18);sheet=Image.new('RGB',(1440,900),(12,12,14));d=ImageDraw.Draw(sheet)
    for i,shot in enumerate(m['shots']):
        im=Image.open(out/'review'/f"{shot['id']}.png");im.thumbnail((480,270));x=(i%3)*480;y=(i//3)*300;sheet.paste(im,(x,y));d.text((x+16,y+277),shot['id'].replace('_',' ').upper(),font=font,fill=(180,183,190))
    sheet.save(out/'review/contact_sheet.jpg',quality=94)

def page():
    font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',26);sheet=Image.new('RGB',(1920,630),(8,8,10));d=ImageDraw.Draw(sheet)
    for i,finish in enumerate(['silver','black']):
        path=BASE/finish/'poster.jpg'
        if path.exists():im=Image.open(path);im.thumbnail((960,540));sheet.paste(im,(i*960,0));d.text((i*960+40,567),'SILVER / Brushed aluminium' if i==0 else 'BLACK / Anodized aluminium',font=font,fill=(207,210,216))
    sheet.save(BASE/'finish_comparison.jpg',quality=96)
    html='''<!doctype html><html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Viscous — The art of articulation</title><style>*{box-sizing:border-box}body{margin:0;background:#080809;color:#eee;font:16px/1.6 system-ui,sans-serif}main{max-width:1400px;margin:auto;padding:70px 5vw}small{letter-spacing:.2em;color:#9a9da5}h1{font-weight:500;letter-spacing:-.05em;font-size:clamp(44px,7vw,90px);line-height:1.05;margin:22px 0}h2{font-weight:450}p{color:#a8abb3}a{color:#dce3ed;text-underline-offset:5px}video,img{width:100%;display:block;border-radius:8px}section{margin:64px 0}.grid{display:grid;grid-template-columns:1fr 1fr;gap:28px}.links{display:flex;gap:20px;flex-wrap:wrap}.shots{display:grid;grid-template-columns:repeat(3,1fr);gap:18px}.shots p{font-size:12px}button{border:1px solid #555;background:#17171a;color:#eee;padding:12px 18px;border-radius:30px;cursor:pointer}@media(max-width:760px){.grid,.shots{grid-template-columns:1fr}}</style><main><small>VISCOUS / LAUNCH FILM</small><h1>The art of<br>articulation.</h1><p>One arm. Two expressions. A 32-second study in form and motion.</p><section><img src="finish_comparison.jpg" alt="Silver and anodized black Viscous arm finishes side by side"></section><div class="grid">'''
    for finish in ['silver','black']:
        html+=f'<article><h2>{finish.capitalize()}</h2><video controls playsinline preload="metadata" poster="{finish}/poster.jpg"><source src="{finish}/viscous_launch_web.mp4" type="video/mp4"></video><p class="links"><a href="{finish}/viscous_launch.mp4" download>1080p master</a><a href="{finish}/viscous_launch_web.mp4" download>Web copy</a><a href="{finish}/viscous_launch.blend" download>Blender scene</a><a href="{finish}/poster.jpg">Poster</a></p></article>'
    html+='</div><p><button id="compare">Play both in sync</button> <button id="pause">Pause both</button></p>'
    for finish in ['silver','black']:
        mpath=BASE/finish/'motion.json'
        if not mpath.exists():continue
        m=json.loads(mpath.read_text());html+=f'<section><h2>{finish.capitalize()} / shot studies</h2><div class="shots">'
        for shot in m['shots']:html+=f'<a href="{finish}/review/{shot["id"]}.png"><img loading="lazy" src="{finish}/review/{shot["id"]}.png" alt="{finish} {shot["id"]}"><p>{shot["id"].replace("_"," ").upper()} · {(shot["start"]-1)/24:g}–{shot["end"]/24:g}s</p></a>'
        html+='</div></section>'
    html+='''<p><a href="README.md">Production notes and rebuild instructions</a> · <a href="reference/STYLE_NOTES.md">Reference study</a></p></main><script>const films=[...document.querySelectorAll('video')];document.querySelector('#compare').onclick=()=>{films.forEach((v,i)=>{v.currentTime=0;v.muted=i>0;v.play().catch(()=>{})})};document.querySelector('#pause').onclick=()=>films.forEach(v=>v.pause());</script></html>'''
    (BASE/'index.html').write_text(html)
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--finish',choices=['silver','black','both'],default='both');p.add_argument('--package-only',action='store_true');a=p.parse_args()
    if not a.package_only:
        for f in (['silver','black'] if a.finish=='both' else [a.finish]):encode(f)
    page()
