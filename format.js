// 参考 https://stackoverflow.com/a/68764864 并修改

// 去掉符号d f .
let Format = function (o, f) {
	/* implements things using (Number).toFixed:
       ${1/3}{} -> 0
       ${1/3}{%} -> 33%
       ${1/3}{.2} -> 0.33
       ${1/3}{.3%} -> 33.333%
	   ${1/3}{2} -> .33
	   ${4/3}{2} -> 1.3
	   ${0.99}{1} -> 1
	   ${9.999}{3} -> 10.0
	   ${99.99}{3} -> 100
	   ${999.9}{3} -> 1000
	   ${1/3}{02} -> 00
	   ${1/3}{ 2} ->  0
       ${{a:1/3,b:1/3}}{2} -> {"a":0.33, "b":0.33}
       ${[1/3,1/3]}{2} -> [0.33, 0.33]
    */
	if (f === undefined) return typeof o === 'string' ? o : JSON.stringify(o)
	else if (o instanceof Array) return '[' + o.map(x => Format(x, f)) + ']'
	else if (typeof o === 'object' && o !== null /*&&!Array.isArray(obj)*/)
		return JSON.stringify(
			Object.fromEntries(Object.entries(o).map(([k, v]) => [k, Format(v, f)]))
		)

	let [match, pad, digi = '0', perc, v] = f.match(/^([ 0.])?([1-9]\d*)?(%)?$/) ?? []
	if (match == null) throw 'error format ' + f
	if (o == null || o != o) return '' + o

	if (perc) o *= 100
	digi = parseFloat(digi)
	if (pad == '.') o = o.toFixed(digi)
	else if (pad != null) o = o.toFixed().padStart(digi, pad)
	else {
		let neg = o < 0 ? '-' : ''
		neg && (o = -o)
		if ((v = o.toFixed(digi)).startsWith('0.')) o = neg + v.slice(1)
		else if ((v = o.toFixed()).length >= digi) o = neg + v
		else o = neg + o.toPrecision(digi)
	}
	return o + (perc ? '%' : '')
}

let _ = (globalThis._ = function (strs, ...args) {
	/* usage: F`Demo: 1+1.5 = ${1+1.5}{.2f}` 
          --> "Demo: 1+1.5 = 2.50" 
    */
	let R = strs[0]
	args.forEach((arg, i) => {
		let fmt = strs[i + 1].match(/^\{(.*)\}/)
		R += Format(arg, fmt?.[1]) + strs[i + 1].slice(fmt?.[0].length)
	})
	return R
})
console._ = (strs, ...args) => console.log(_(strs, ...args))
