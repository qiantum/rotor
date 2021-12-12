// 参考 https://stackoverflow.com/a/68764864 并修改

// 去掉符号d f .
let FORMATTER = function (o, f) {
	/* implements things using (Number).toFixed:
       ${1/3}{2} -> 0.33
       ${1/3}{%} -> 33%
       ${1/3}{3%} -> 33.333%
       ${1/3}{} -> 0
	   ${1/3}{02} -> 00
	   ${1/3}{ 2} ->  0
       ${{a:1/3,b:1/3}}{2} -> {"a":0.33, "b":0.33}
       ${[1/3,1/3]}{2} -> [0.33, 0.33]
    */
	let match, pad, digi, perc
	if (f === undefined) {
		if (typeof o === 'string') return o
		else return JSON.stringify(o)
	} else if (o instanceof Array) return '[' + o.map(x => FORMATTER(x, f)) + ']'
	else if (typeof o === 'object' && o !== null /*&&!Array.isArray(obj)*/)
		return JSON.stringify(
			Object.fromEntries(Object.entries(o).map(([k, v]) => [k, FORMATTER(v, f)]))
		)
	else if ((match = f.match(/^([ 0])?([1-9]\d*)?(%)?$/))) [match, pad, digi = '0', perc] = match
	else throw 'error format ' + f

	if (o === null) return 'null'
	if (o === undefined) return 'undefined'

	if (perc) o *= 100
	digi = parseFloat(digi)
	if (pad == null) o = o.toFixed(digi)
	else o = o.toFixed().padStart(digi, pad)
	return o + (perc ? '%' : '')
}

let _ = (globalThis._ = function (strs, ...args) {
	/* usage: F`Demo: 1+1.5 = ${1+1.5}{.2f}` 
          --> "Demo: 1+1.5 = 2.50" 
    */
	let R = strs[0]
	args.forEach((arg, i) => {
		let fmt = strs[i + 1].match(/^\{(.*)\}/)
		R += FORMATTER(arg, fmt?.[1]) + strs[i + 1].slice(fmt?.[0].length)
	})
	return R
})
console._ = (strs, ...args) => console.log(_(strs, ...args))
