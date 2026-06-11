import pprint, pdb
from qr_utils.misc_utils import current_log_dir, current_log_file_num
from collections.abc import Sequence

# debug_on  : print to tty
# pause_on : pause tty
# log_on : print to log

debug_on = []
pause_on = []
log_on = {}   # This is a dictionary, mapping tags to indentation
draw_on = []

IN_HEURISTIC = False

be_silent = False
def silent(status):
    global be_silent
    be_silent = status

do_not_log = False
def no_log(status):
    global do_not_log
    do_not_log = status
    
def debug_add(tags):
    if is_l_or_t(tags):
        debug_on.extend(tags)
    else:
        debug_on.append(tags)
def debug_remove(tags):
    if is_l_or_t(tags):
        for x in tags: debug_on.remove(x)
    else:
        debug_on.remove(tags)
def debug_set(tags):
    global debug_on
    debug_on = tags[:]
def debug_get():
    global debug_on
    return debug_on
def pause_add(tags):
    if is_l_or_t(tags):
        pause_on.extend(tags)
    else:
        pause_on.append(tags)
def pause_set(tags):
    global pause_on
    pause_on = tags[:]
def log_set(tags):
    global log_on
    log_on = tags.copy()

def update_log_tags(tags):
    for tag in tags:
        if tag in log_on:
            tags[tag] = log_on[tag]
            del(log_on[tag])

def draw_add(tags):
    if is_l_or_t(tags):
        draw_on.extend(tags)
    else:
        draw_on.append(tags)            

# Add any tag that is not already in log_on to log_on.
def cleanup_log_tags(tags):
    for tag in tags:
        if tags[tag] is not None:
            log_on[tag] = tags[tag]    
def log_add(tags):
    if isinstance(tags, dict):
        log_on.update(tags)
        return
    if isinstance(tags, str):
        tags = [tags]
    for tag in tags:
        log_on[tag] = 1
def log_remove(tags):
    if isinstance(tags, str):
        tags = [tags]    
    for tag in tags:
        log_on.pop(tag, None)
        

# tag lists contain: *, symbol, or (symbol, symbol)
# tag in a tr statement can be: symbol or (symbol, symbol)

# keys is a dictionary
# Possible keywords:  pause
#    pause: pauses
#    ol: write onto single line, if true
#    skip: skip this whole thing
#    h: do it in the heuristic 

# Return true if this tag should be traced, given a particular list of tags
def traced(genTag, tags, skip = False, keys = None):
    result = (not skip) and \
      ((not IN_HEURISTIC) or
       ('debugInHeuristic' in debug_on) or ('debugInHeuristic' in tags) or
       ((keys is not None) and keys.get('h', False))) and \
       (genTag == '*'  or  ('*' in tags) or
        (any(tag in tags for tag in genTag) if is_l_or_t(genTag)
         else (genTag in tags)))
    assert result != {}, 'No tags for '+str(genTag)
    return result

# Always print and log this.  
def tr_always(*msg, **keys):
    keys['h']=True                        # do this in the heuristic as well
    tr('*', *msg, **keys)
tr_a = tr_always

def has_tag(test, tag_or_list):
    return (test == tag_or_list or
            (is_l_or_t(tag_or_list) and test in tag_or_list)) 

def is_l_or_t(thing):
    """ Return true if thing is a list or a tuple """
    return isinstance(thing, (list, tuple))

# Decide whether to write into log
def log(tag, **keys):
    return has_tag('log', tag) or traced(tag, log_on, False, keys)

# Decide whether to write into Constance log
def constance(tag, **keys):
    return has_tag('constance', tag)

# Decide whether to write to tty
def debug(tag, **keys):
    return has_tag('terminal', tag) or traced(tag, debug_on, False, keys)

# Decide whether to draw to tty
def draw(tag, **keys):
    return has_tag('draw', tag) or traced(tag, draw_on, False, keys)

# Decide whether to pause tty.  Default to no, even if '*'
def pause(tag, **keys):
    return keys.get('pause', False) or \
      ((tag != '*') and traced(tag, pause_on, False, keys))

def debug_msg(tag, *msgs):
    if debug(tag):
        print(tag, ':')
        for m in msgs:
            print('    ', m)
    if pause(tag):
        input(tag+'-Go?')

def debug_msg_skip(tag, skip = False, *msgs):
    if debug(tag, skip):
        print(tag, ':')
        for m in msgs:
            print('    ', m)
    if pause(tag, skip):
        input(tag+'-Go?')

log_gen = '%s/a%s.log'
log_gen_C = '%s/aC%s.log'
log_gen_H = '%s/aH%s.log'

def tr(genTag, *msg, **keys):
    if keys.get('skip', False) or 'noTrace' in debug_on:  return

    doLog = not do_not_log and log(genTag, **keys)
    doConstance = not do_not_log and constance(genTag, **keys)
    doDebug = not be_silent and debug(genTag, **keys)
    doPause = not be_silent and pause(genTag, **keys)
    # between parts of msg
    sep = ' ' if keys.get('ol', True) else '\n'
    # at the end of msg
    end = keys.get('end', '\n')

    real_tags = (set(genTag) if is_l_or_t(genTag) else {genTag}) - {'*', 'log', 'terminal', 'draw', 'pause'}
    if len(real_tags) == 0:
        tagStr = ''
    elif len(real_tags) == 1:
        tagStr = str(real_tags.pop()) + ': '
    else:
        tagStr = str(real_tags) + ': '


    ppr = keys.get('ppr', False)            # pretty print

    # Logging text
    if doLog and msg:
        if current_log_file_num() is None:
            num = '_prep'
        else:
            num = current_log_file_num()
        if IN_HEURISTIC:
            targetFile = log_gen_H%(current_log_dir(), str(num))
        else:
            targetFile = log_gen%(current_log_dir(), str(num))

        with open(targetFile, 'a') as f:
            tag_depth = log_on.get(genTag, 0)
            indent = tag_depth*'    '
            f.write(indent)
            for m in msg:
                if ppr:
                    pprint.pprint(m, indent=4, stream=f)
                    f.write(sep)
                else:
                    f.write(str(m) + sep)
            f.write('\n') 


    if doConstance and msg:
        if current_log_file_num() is None:
            num = '_prep'
        else:
            num = current_log_file_num()

        targetFileC = log_gen_C%(current_log_dir(), str(num))

        with open(targetFileC, 'a') as f:
            if isinstance(genTag, str):
                tag_depth = log_on.get(genTag, 0)
            elif isinstance(genTag, (list, tuple)):
                tag_depth = max([log_on.get(tag, 0) for tag in genTag])
            else:
                tag_depth = 0
            indent = tag_depth*'    '
            f.write(indent)
            for m in msg:
                if ppr:
                    pprint.pprint(m, indent=4, stream=f)
                    f.write(sep)
                else:
                    f.write(str(m) + sep)
            f.write('\n') 
            
    # Printing to console
    if (doDebug or doPause) and msg:
        # if tagStr not in {'terminal: ', 'log: ', 'pause: ', 'draw: '}: 
        #     print(tagStr, end=' ')
        for m in msg:
            if ppr:
                pprint.pprint(m, indent=4)
                print('', end=sep)
            else:
                print(m, end=sep)
        if end: print(end, end='')

    # Pausing
    if doPause:
        if keys.get('bell', False) is True:
            print('\a')
        input(tagStr+' go?')


