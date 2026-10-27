"""Event-triggered local llama.cpp multimodal inference with constrained output."""
import base64
import json
import time
import urllib.request
from .contracts import SCHEMA,validate

def interpret(text,image_png=None,url='http://127.0.0.1:8088/v1/chat/completions'):
    if not isinstance(text,str) or not 0<len(text)<=500:raise ValueError('INVALID_TEXT')
    # The 500M model is more reliable with a short request following concrete
    # demonstrations than with a long mixed-language instruction paragraph.
    content=[{'type':'text','text':text}]
    if image_png:content.insert(0,{'type':'image_url','image_url':{'url':'data:image/png;base64,'+base64.b64encode(image_png).decode()}})
    examples=[('查询状态 / Show robot status',dict(skill='inspect',object_id='none',target_id='none',arm='none')),
        ('前往北侧 / Navigate to the north entry',dict(skill='navigate',object_id='none',target_id='north_entry',arm='none')),
        ('前往南侧 / Navigate to the south entry',dict(skill='navigate',object_id='none',target_id='south_entry',arm='none')),
        ('左臂回位 / Move the left arm home',dict(skill='home',object_id='none',target_id='none',arm='left')),
        ('右臂回位 / Move the right arm home',dict(skill='home',object_id='none',target_id='none',arm='right')),
        ('暂停 / Pause the robot',dict(skill='pause',object_id='none',target_id='none',arm='none'))]
    messages=[]
    for sentence,answer in examples:
        messages.extend([{'role':'user','content':sentence},{'role':'assistant','content':json.dumps(answer)}])
    messages.append({'role':'user','content':content})
    payload={'messages':messages,'temperature':0,'max_tokens':110,
             'response_format':{'type':'json_object','schema':SCHEMA}}
    start=time.monotonic()
    request=urllib.request.Request(url,data=json.dumps(payload).encode(),headers={'Content-Type':'application/json'})
    with urllib.request.urlopen(request,timeout=45) as r:response=json.load(r)
    proposal=validate(json.loads(response['choices'][0]['message']['content']))
    return dict(proposal=proposal,source='SmolVLM-500M-Q8',latency_seconds=time.monotonic()-start,
                image_supplied=bool(image_png),usage=response.get('usage',{}),runtime_ready=False)


def review_feedback(context,image_png=None,url='http://127.0.0.1:8088/v1/chat/completions'):
    """One real model call per checkpoint; no hidden rule fallback or reasoning text."""
    from .feedback_review import REVIEW_SCHEMA,validate_review
    instruction=('Review the next robot step using the measured feedback. Return one decision. '
        'The current JSON is authoritative. If all_steps_done is false, sensors_fresh is true, robot_ready is true, fault is empty, active_task is null and candidate is not null, the ONLY valid answer is exactly {"decision":"dispatch"}. '
        'Fault or previous FAILED/CANCELLED/INTERRUPTED/MISSING: stop. Stale sensors or robot not ready or active task: wait. '
        'If all_steps_done is true and previous_status is SUCCEEDED, the ONLY valid answer is exactly {"decision":"complete"}. Never dispatch when all_steps_done is true. '
        'Image is the current head camera, not a localization estimate. '
        'The candidate is operator-requested; do not invent tasks.')
    examples=[({'sensors_fresh':True,'robot_ready':True,'fault':'','active_task':None,'previous_status':'NONE','all_steps_done':False,'candidate':'navigate'},'dispatch'),
              ({'sensors_fresh':True,'robot_ready':True,'fault':'','active_task':None,'previous_status':'SUCCEEDED','all_steps_done':True,'candidate':None},'complete')]
    messages=[{'role':'system','content':instruction}]
    for sample,decision in examples:
        messages.extend([{'role':'user','content':json.dumps(sample)},{'role':'assistant','content':json.dumps(dict(decision=decision))}])
    content=[{'type':'text','text':json.dumps(context,ensure_ascii=False)}]
    if image_png:content.insert(0,{'type':'image_url','image_url':{'url':'data:image/png;base64,'+base64.b64encode(image_png).decode()}})
    messages.append({'role':'user','content':content})
    payload=dict(messages=messages,temperature=0,max_tokens=30,response_format={'type':'json_object','schema':REVIEW_SCHEMA})
    start=time.monotonic();request=urllib.request.Request(url,data=json.dumps(payload).encode(),headers={'Content-Type':'application/json'})
    with urllib.request.urlopen(request,timeout=60) as response:raw=json.load(response)
    output=raw['choices'][0]['message']['content'];value=validate_review(json.loads(output))
    return dict(**value,raw_output=output,source='SmolVLM-500M-Q8',latency_seconds=time.monotonic()-start,
                image_supplied=bool(image_png),usage=raw.get('usage',{}),system_instruction=instruction,
                few_shot_examples=examples,context=context,runtime_ready=False)
