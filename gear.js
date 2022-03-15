// Copyright: Qianyan Cai
// License: GPL v3

// 渐开线齿轮 Involute Gear
function GearI({
	M, // 模数
	A = 20, // 压力角度数
	Z, // 齿数
	S = 0, // 变位系数
	tickn = 240, // 齿廓总步进数
	size, // 预估像素
}) {
	if ((M = round(M * 8) >> 3) <= 0) throw 'err M'
	if (!((A = roundepsi(A)) > 0)) throw 'err A'
	if (Z != (Z |= 0) || Z < 4) throw 'err Z'
	if (!((S = roundepsi(S)) >= 0)) throw 'err S'
	tickn = max(ceil(tickn / Z / 2), 3) * Z * 2 // 齿廓步进数，齿数整倍数
	size = ceil(+size || min(size.width, size.height))

	let TA = (A * PI) / 180 // 压力角
	let G = M * 0.5 * Z // 节圆半径
	let B = G * cos(TA) // 基圆半径
	let R = M * 0.5 * (Z + S) // 分度圆半径
	if (R < B) throw `conflict: R ${R} < B ${B}`
	let U = R + M // 齿顶半径
	let F = R - M * 1.25 // 齿根半径

	let TP = PI2 / Z // 齿距角（依据分度点）
	let TR = PI / Z // 分度角（依据分度点）
	let CR = sqrt(R * R - B * B) / B // 分度渐开角
	let CU = sqrt(U * U - B * B) / B // 齿顶渐开角
	let CF = sqrt(max(F * F - B * B, 0)) / B // 齿根渐开角
	let TC = ((CU - CF) * Z * 2) / tickn // 步进渐开角
	let TRB = CR - acos(B / R) // 分度起始角（依据基点）
	let TZ = z => (z * PI2) / Z - TRB // 齿起始角（依据分度点）
	let TB = TR + TRB + TRB // 基角（依据基点）

	let CX = (T, C) => B * cos(T + C) + B * C * sin(T + C) // 齿廓点X
	let CY = (T, C) => B * sin(T + C) - B * C * cos(T + C) // 齿廓点Y
	let FX = T => (F >= B ? CX(T, CF) : F * cos(T)) // 齿根X
	let FY = T => (F >= B ? CY(T, CF) : F * sin(T)) // 齿根Y

	Object.assign(this, { M, A, Z, S, G, B, R, U, F })
	Object.assign(this, { TP, TZ })

	// 参数显示
	function params(T) {
		let p = ''
		// if (T != null) {
		// 	p = _`|${(RBC(T)[0] / PI2) * 360}{02}__`
		// 	let a = (T / TS(1)) * PI
		// 	let pis = (3 + 1 - sqrt(3 * 3 - sin(a) * sin(a)) - cos(a)) / 2
		// 	p += _`${VS(T) / 100}{03}:${VS(T) / V}{.2}|${(1 - cos(a)) / 2}{.2}|${pis}{.2}`
		// }
		return _`M${M} A${A}__Z${Z} S${S}__B${B}{1} R${R}{1} U${U}{1} F${F}{1}__`
		// _`Z${Z}__E${E}{}__P${P}{}__K${K}{1}__` +
		// _`V${V / 100}{} ${VV / 100}{}__${VB / 100}{} ${VN / 100}{}__` +
		// _`RB${GB}{.2} C${(RBCC / PI2) * 360}{}` +
		// p
	}
	console.log(...params().split('__'), _`tn${tickn}`, F < B ? 'F<B' : '')

	size /= R + R + M * 2.5
	this.size = ceil(U * size * 2)
	this.Rsize = R * size

	this.$ = ({ canvas, midx, midy, param }) => {
		let $ = canvas.getContext('2d')
		let x = midx ?? canvas.width / 2 // 齿心X
		let y = midy ?? canvas.height / 2 // 齿心Y
		let $param = T => (param.textContent = params(T).replace(/__/g, '\n'))

		function $$({ color = '#000', opa = '', thick = 1, dash } = {}, fill) {
			$.beginPath(), ($.lineWidth = thick)
			opa.length == 1 && /#....../.test(color) && (opa += opa)
			opa.length > 0 && /#....(....)?$/.test(color) && (color = color.slice(0, -opa.length))
			fill ? ($.fillStyle = color + opa) : ($.strokeStyle = color + opa)
			dash && $.setLineDash(dash)
		}
		function $$$(fill) {
			fill ? $.fill() : $.stroke(), ($.lineWidth = 1)
			fill ? ($.fillStyle = '#000') : ($.strokeStyle = '#000')
			$.setLineDash([])
		}
		// 画齿心
		function $O(style) {
			$$({ color: '#ccc', ...style })
			$.arc(x, y, 1, 0, PI2), $$$()
		}
		// 画分度圆
		function $R(style) {
			$$({ color: '#ccc', ...style })
			$.arc(x, y, R * size, 0, PI2), $$$()
		}
		// 画基圆
		function $B(style) {
			$$({ color: '#999', dash: [1, 3], ...style })
			$.arc(x, y, B * size, 0, PI2), $$$()
		}
		// 画齿
		function $C(T, z = 0, style, f = true) {
			$$({ color: '#000', ...style })
			T += TZ(z)
			$.moveTo(x + FX(T) * size, y + FY(T) * size)
			for (let C = CF; C <= CU + EPSI; C += TC)
				$.lineTo(x + CX(T, C) * size, y + CY(T, C) * size)
			T += TB
			for (let C = CU; C >= CF - EPSI; C -= TC)
				$.lineTo(x + CX(T, -C) * size, y + CY(T, -C) * size)
			$.lineTo(x + FX(T) * size, y + FY(T) * size)
			if (f) $.lineTo(x + FX(T - TB + TP) * size, y + FY(T - TB + TP) * size)
			$$$()
		}
		// 画全部齿
		function $CZ(T, style) {
			for (let z = 0; z < Z; z++) $C(T, z, style)
		}
		return Object.assign({ param: $param, x, y, O: $O, R: $R, B: $B, C: $C, CZ: $CZ })
	}
}
