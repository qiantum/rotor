// Copyright: Qianyan Cai
// License: GPL v3

// 渐开线齿轮 Involute Gear
function GearInv({
	M, // 模数
	A = 20, // 压力角度数
	Z, // 齿数
	S = 0, // 变位系数
	I = false, // 内齿轮
	T0 = 0, // 啮合起始角 >=0 从齿顶起始 <0 从齿根起始
	tickn = 240, // 齿廓总步进数
}) {
	if (!((M = roundepsi(M)) > 0)) throw 'err M'
	if (!((A = roundepsi(A)) > 0)) throw 'err A'
	if (Z != (Z |= 0) || Z < 6) throw 'err Z'
	tickn = max(ceil(tickn / Z / 2), 3) * Z * 2 // 齿廓步进数，齿数整倍数

	let AA = (A * PI) / 180 // 压力角
	let TP = PI2 / Z // 齿距角
	let TQ = PI / Z // 齿顶根角
	let TB = (PI + 4 * S * tan(AA)) / Z + GearInv.AT(AA) * 2 // 基角（依据基点，齿厚角+两侧分度展角）
	TB < EPSI && (TB = EPSI)

	let P = M * Z * 0.5 // 分度圆半径
	let B = P * cos(AA) // 基圆半径
	let U = P + M * (S + (I ? 1.25 : 1)) // 齿顶半径
	let F = P + M * (S - (I ? 1 : 1.25)) // 齿根半径
	let fixF = max(0, (I ? B : M) - F)
	F += fixF
	// let Q = P + M * S // 中分圆半径
	// U = Q + M * (I ? 1.25 : 1) // 齿顶半径
	// F = Q - M * (I ? 1 : 1.25) // 齿根半径
	// let CQ = sqrt(Q * Q - B * B) / B // 中分渐开角
	// TB = TQ + (CQ - acos(B / Q)) * 2 // 基角（依据基点，中分齿厚角+两侧中分展角）

	let fixU = 0
	if (S > 0) {
		let u = U // 齿顶交叉修正
		for (; GearInv.RT(u, B) > TB / 2; u -= M / 32);
		fixU = u - U
		if (fixU) (U = u), (F += fixU), (fixF += fixU)
		let f = max(F, B) // 齿根交叉修正
		for (; GearInv.RT(f, B) < (TB - TP) / 2; f += M / 32);
		if (f != F && f != B) (fixF += f - F), (F = f)
	}
	let CU = GearInv.RC(U, B) // 齿顶渐开角
	let CF = GearInv.RC(F, B) // 齿根渐开角
	let Ct = max(((CU - CF) * Z * 2) / tickn, EPSI) // 步进渐开角
	let CX = (T, C) => B * cos(T + C) + B * C * sin(T + C) // 齿廓点X
	let CY = (T, C) => B * sin(T + C) - B * C * cos(T + C) // 齿廓点Y
	let TZ = z => (z * PI2) / Z - TB / 2 + T0 + (T0 < 0 ? TQ : 0) // 齿起始角（依据基点）

	let T2 = (T, gear2) => (T / gear2.Z) * Z * (I == gear2.I ? -1 : 1)
	let G2 = (E, gear2) => (E * Z) / max(abs(Z + gear2.Z * (I == gear2.I ? 1 : -1)), 1)

	Object.assign(this, { M, A, Z, S, I, B, P, U, F })
	Object.assign(this, { TP, TQ, TZ, T2, G2 })

	// 参数显示
	function params(T) {
		return _`M${M}{3} A${A}__Z${Z} S${S}{2}__B${B}{.1} P${P}{.1}__F${F}{.1} U${U}{.1}__`
	}
	console.log(...params().split('__'), _`${TB}{5} tn${tickn}`, _`F${fixF}{.1} U${fixU}{.1}`)

	this.$ = ({ canvas, midx = 0, midy = 0, x, y, zoom = 1, param }) => {
		x ??= canvas.width / 2 + midx // 齿心X
		y ??= canvas.height / 2 + midy // 齿心Y
		let $param = T => (param.textContent = params(T).replace(/__/g, '\n'))
		let $ = canvas.getContext('2d')

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
		function $O(T, O = M * 0.75 * zoom, style) {
			$$({ color: '#ccc', ...style })
			$.moveTo(x, y), $.arc(x, y, O, -T - T0, PI2 - T - T0), $$$()
		}
		// 画基圆
		function $B(style) {
			$$({ color: '#ccc', ...style })
			$.arc(x, y, B * zoom, 0, PI2), $$$()
		}
		// 画节圆
		function $G(G, style) {
			$$({ color: '#999', dash: [1, 3], ...style })
			$.arc(x, y, G * zoom, 0, PI2), $$$()
		}
		// 画齿
		function $C(T, z = 0, style, f = true) {
			$$({ color: '#000', ...style })
			let to
			T += TZ(z)
			for (let C = CF; C <= CU + EPSI; C += Ct)
				(to = to ? $.lineTo : $.moveTo).call($, x + CX(T, C) * zoom, y - CY(T, C) * zoom)
			for (let C = CU; C >= CF - EPSI; C -= Ct)
				$.lineTo(x + CX(T + TB, -C) * zoom, y - CY(T + TB, -C) * zoom)
			if (f) {
				if (F < B)
					$.lineTo(x + F * cos(T + TB) * zoom, y - F * sin(T + TB) * zoom),
						$.lineTo(x + F * cos(T + TP) * zoom, y - F * sin(T + TP) * zoom)
				$.lineTo(x + CX(T + TP, CF) * zoom, y - CY(T + TP, CF) * zoom)
			}
			$$$()
		}
		// 画全部齿
		function $CZ(T, style) {
			for (let z = 0; z < Z; z++) $C(T, z, style)
		}
		return { param: $param, x, y, O: $O, B: $B, G: $G, C: $C, CZ: $CZ }
	}
}
GearInv.AT = A => tan(A) - A // 压力角求展角（展角+压力角=渐开角）
GearInv.RC = (R, B) => sqrt(max(R * R - B * B, 0)) / B // 半径求渐开角
GearInv.RT = (R, B) => GearInv.AT(acos(B / R)) // 半径求展角

GearInv.M = (ZZ, E, EE) => E / (ZZ * 0.5 + EE) // 中心距 求 模数
GearInv.E = (M, ZZ, EE) => M * (ZZ * 0.5 + EE) // 变距系数 求 中心距
GearInv.EE = (M, ZZ, E) => E / M - ZZ * 0.5 // 中心距 求 变距系数
GearInv.EEmin = (A, ZZ) => (cos((A * PI) / 180) - 1) * ZZ * 0.5 // 最小变距系数
// 变距系数 求 变位系数和差
GearInv.EESS = function (A, ZZ, EE) {
	if (ZZ <= 0) return 0
	A = (A * PI) / 180
	ZZ *= 0.5
	let AW = acos(cos(A) / (EE / ZZ + 1))
	return ((GearInv.AT(AW) - GearInv.AT(A)) / tan(A)) * ZZ
}
