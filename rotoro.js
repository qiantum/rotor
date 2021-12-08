let { min, max, abs, floor, ceil, round, PI, cos, sin, sqrt } = Math
let PI2 = (Math.PI2 = PI + PI)
let EPSI = (Number.EPSILON = 1 / (1 << 12))
let atan, diff, diffAbs, cross, area

function Rotor({
	N, // 顶角数
	E, // 偏心距
	P = (N - 1) * 0.8, // 转子顶半径 / 偏心距
	Q = P, // 转子腰半径 / 偏心距
	BP = 0.1, // 缸体转子间隙 / 偏心距
	Tick: Tickn = 16, // 顶间旋转步进
	Rfast = true, // 快速计算转子型线
	size, // 预估像素
}) {
	let PIN = PI / N
	let N2 = N + N
	Tickn = floor(min(Tickn * N2, 160) / N2) * N2 // 顶间旋转步进
	size = ceil(+size || min(size.clientWidth, size.clientHeight))
	E ??= floor(size / (2.3 * (P + N + 3))) // 偏心距
	let G = E * N // 转子大节圆半径
	let g = G - E // 曲轴小节圆半径
	P = E * (P + N + 2) // 转子顶半径
	Q = E * (Q + N) // 转子腰半径
	BP *= E // 缸体转子间隙

	let Tn = n => (n * PI2) / N
	let Tst = st => (st * PI) / (N - 1)
	// 旋转步进角
	let Tick_ = (this.Tick_ = [...Array.seq(0, Tickn)].map(t => (t / Tickn) * PI2))
	let Tick = (this.Tick = Tick_.slice(0, Tickn))

	// 缸体型线
	let BB = Tick_.map(T => [
		E * cos(T * N - PI) + (P + BP) * cos(T),
		E * sin(T * N - PI) + (P + BP) * sin(T),
	])
	size = BB.reduce((v, [X, Y]) => max(v, abs(X), abs(Y)), 0)
	// 缸体腰线
	let BQt = [...Array.seq(Tickn - Tickn / N2, Tickn - 1), ...Array.seq(0, Tickn / N2)]
	// 缸体顶线
	let TBP = Tst(1)
	let BPt = [...Array.seq(Tick.binFind(TBP - PIN), Tick.binFind(TBP + PIN))]

	// 缸体对转子旋转
	function* BTR(TT, BT) {
		for (let T of TT)
			for (let B of BT) {
				let X = P * cos(B + T) - E * cos(B * N + T) - E * cos(T * (N - 1))
				let Y = P * sin(B + T) - E * sin(B * N + T) + E * sin(T * (N - 1))
				let A = atan(X, Y) // [0,PI2) B==0 沿T严格递增 B>0 沿T循环严格递增
				yield [A, sqrt(X * X + Y * Y), X, Y]
			}
	}
	// 转子型线、即缸体绕转子心的内包络线
	let R
	if (Rfast) {
		let dot = T => BTR([T], BQt.map(Tick.At()))
		R = MinDot(0, Tickn, dot, (T, t) => (t % (Tickn / N2) ? null : t % (Tickn / N) ? P : Q))
	} else {
		let S = [...BTR(Tick, [0])]
		for (let B of Tick.slice(1)) {
			// 缸体转动
			let bb = [...BTR(Tick, [B])]
			bb.forEach(b => (b[4] = S.binFind(b[0], 0, -1)))
			let M = []
			// 对缸体每段
			for (let bt of Tick.keys()) {
				let [a_, r_, x_, y_, t_] = bb[bt]
				let [a, r, x, y, t] = bb[bt + 1] || bb[0]
				t_--, (t %= S.length) // 对应转子连续段
				for (let T_ = t_, T1, T; T_ != t; T_ = T) {
					let [A_, R_, X_, Y_] = S[T_]
					let [A, R, X, Y] = S[(T = T1 = T_ + 1)] || S[(T = 0)]
					// 求交点和半径
					let [SX, SY, abc, abd, cda, cdb] = cross(X_, Y_, X, Y, x_, y_, x, y)
					if (T != t && cdb < -EPSI && T) M.push([1 / 0, 0, 0, 0, T]) // 去掉长半径转子点
					if (T_ == t_ && abc > EPSI) M.push([a_, r_, x_, y_, T1]) // 短半径缸体点
					if (T == t && abd > EPSI) M.push([a, r, x, y, T1]) // 短半径缸体点
					if (SX != null) M.push([atan(SX, SY), sqrt(SX * SX + SY * SY), SX, SY, T1]) //交点
				}
			}
			M.sort((s1, s2) => s1[4] - s2[4] || s1[0] - s2[0]) // 保持角度有序
			let t = 0 // 加减点
			for (let m of M)
				if (m[0] == 1 / 0) S.splice(m[4] + t--, 1)
				else S.splice(m[4] + t++, 0, m), m.length--
			S[S.push([...S[0]]) - 1][0] += PI2 // 闭合
			t = 0 // 合并重合点
			for (let R of S)
				if (R[0] < S[t][0]) throw 'err a'
				else if (R[0] - S[t][0] > EPSI && abs(R[1] - S[t][1]) > EPSI) S[++t] = R
			S.length = ++t
		}

		R = function* (T, rt = Array.seq(0, Tickn)) {
			let s = rt.values?.() ?? rt
			let tt = s.next().value
			if (tt == null) return
			let x = E * cos(T * N)
			let y = E * sin(T * N)
			let r = S.binFind(Tick_[tt], 0)
			for (let t of s)
				do
					for (let A, tT = Tick_[t] + (tt > t && PI2); (A = S[r]?.[0]) <= tT; r++)
						yield [x + S[r][1] * cos(T + A), y + S[r][1] * sin(T + A)]
				while (tt > (tt = t) && !(r = 0))
		}
		R.count = S.length - 1
	}
	console._`R in ${R.count} details`

	// 工作容积，总容积，总体积
	let V, K, VV, KK, VB, KB
	{
		let v = area(BQt.map(BB.At())) - area(R(0, BQt))
		V = area(BPt.map(BB.At())) - area(R(TBP, BQt)) - v
		VB = area(BB)
		VV = VB - area(R(0))
		;(K = V / v + 1), (KK = VV / V), (KB = VB / V)
		console._`Vmin ${v}{} Vmax ${V}{} K ${K}{1}  KK ${KK}{1} KB ${KB}{1}`
	}

	Object.assign(this, { N, E, G, g, P, Q, BP, R, V, K, KK, KB, Tn, Tst, size })

	this.$ = ({ canvas, midx, midy, param }) => {
		let $ = canvas.getContext('2d')
		if (param)
			param.textContent =
				_`N${N}{} E${E}{}\nP${P / E}{1} Q${Q / E}{1}\n` + _`K${K}{} ${KK}{1} ${KB}{1}`

		let $x = midx ?? canvas.clientWidth / 2
		let $y = midy ?? canvas.clientHeight / 2
		let $X = T => $x + E * cos(T * N) // 转子心X
		let $Y = T => $y + E * sin(T * N) // 转子心Y

		function $$({ color = '#000', opa = '', thick = 1 } = {}) {
			$.beginPath(), ($.strokeStyle = color + opa), ($.lineWidth = thick)
		}
		function $$$() {
			$.stroke(), ($.strokeStyle = '#000'), ($.lineWidth = 1)
		}
		// 画曲轴小圆
		function $g(style) {
			$$(style), $.arc($x, $y, g, 0, PI2), $$$()
		}
		// 画偏心线
		function $Gg(T, style) {
			$$({ color: '#ccc', thick: 4, ...style })
			$.moveTo($x, $y), $.lineTo($X(T), $Y(T)), $$$()
		}
		// 画转子大圆
		function $G(T, style) {
			$$({ color: '#999', ...style }), $.arc($X(T), $Y(T), G, 0, PI2), $$$()
		}
		// 画转子大圆凸包
		function $GG(style) {
			$$({ color: '#999', opa: '5', ...style }), $.arc($x, $y, G + E, 0, PI2), $$$()
		}
		// 画转子顶
		function $P(T, n = 0, O, style) {
			T += Tn(n)
			$$({ color: '#00f', ...style })
			let X = $X(T) + P * cos(T + PIN)
			let Y = $Y(T) + P * sin(T + PIN)
			BP && $.arc(X, Y, BP, 0, PI2)
			$.moveTo(X, Y), $.lineTo($X(T) + O * cos(T + PIN), $Y(T) + O * sin(T + PIN))
			$$$()
		}
		// 画转子腰
		function $Q(T, n = 0, O, style) {
			T += Tn(n)
			$$({ color: '#0f0', ...style })
			$.moveTo($X(T) + Q * cos(T), $Y(T) + Q * sin(T))
			$.lineTo($X(T) + O * cos(T), $Y(T) + O * sin(T))
			$$$()
		}
		// 画转子全部顶
		function $PN(T, O = G, style) {
			for (let n = 0; n < N; n++) $P(T, n, O?.[n] ?? O?.at?.(-1) ?? O, style)
		}
		// 画转子全部腰
		function $QN(T, O = [0, G], style) {
			for (let n = 0; n < N; n++) $Q(T, n, O?.[n] ?? O?.at?.(-1) ?? O, style)
		}
		// 画转子腰凸包
		function $QQ(style) {
			$$({ color: '#9f9', ...style })
			for (let T of Tick_) {
				let to = T ? $.lineTo : $.moveTo
				to.call($, $X(T) + Q * cos(T), $Y(T) + Q * sin(T))
			}
			$$$()
		}
		// 画缸体
		function $BB(style) {
			$$({ color: '#00f', ...style })
			let to
			for (let [X, Y] of BB) (to = to ? $.lineTo : $.moveTo).call($, $x + X, $y + Y)
			$$$()
		}
		// 画转子
		function $RR(T, style) {
			$$(style)
			let to
			for (let [X, Y] of R(T)) (to = to ? $.lineTo : $.moveTo).call($, $x + X, $y + Y)
			$$$()
		}
		return {
			x: $x,
			y: $y,
			g: $g,
			Gg: $Gg,
			G: $G,
			GG: $GG,
			P: $P,
			Q: $Q,
			PN: $PN,
			QN: $QN,
			QQ: $QQ,
			BB: $BB,
			RR: $RR,
		}
	}

	// 步进点集求最内包络线
	function MinDot(t0, t9, dot, fix) {
		let M = new Array(t9 + 1).fill(1 / 0)
		for (let t = t0; t <= t9; t++)
			for (let [A, D] of dot(Tick_[t], t)) {
				let a = ceil((t9 * A) / PI2)
				if (a >= t0 && a <= t9 && D < M[a]) M[a] = D
			}
		for (let t = t0; t < t9; t++) M[t] = min(M[t], M[t + 1]) * 0.375 + max(M[t], M[t + 1]) * 0.625
		for (let t = t0, d; t <= t9; t++) if ((d = fix(Tick_[t], t)) != null) M[t] = d

		let XY = function* (T, tt = Array.seq(t0, t9)) {
			let x = E * cos(T * N)
			let y = E * sin(T * N)
			for (let t of tt) yield [x + M[t] * cos(T + Tick_[t]), y + M[t] * sin(T + Tick_[t])]
		}
		return (XY.count = M.length - 1), XY
	}
}

atan = function (x, y) {
	if (abs(x) < EPSI) return y > 0 ? PI * 0.5 : PI * 1.5
	return Math.atan(y / x) + (x < 0 ? PI : y < 0 ? PI2 : 0)
}
diff = v => ((v %= PI2) > PI ? v - PI2 : v < -PI ? v + PI2 : v)
diffAbs = v => ((v = abs(v) % PI2) > PI ? PI2 - v : v)

Array.prototype.At = function () {
	return this.at.bind(this)
}
Array.seq = function* (from, to) {
	for (let i = from; i <= to; i++) yield i
}
Array.prototype.binFind = function (v, prop, epsi = EPSI, neg) {
	let l = 0
	for (let h = this.length - 1, m, c; l <= h; ) {
		;(m = (l + h) >>> 1), (c = v - (prop != null ? this[m][prop] : this[m]))
		if (c >= -epsi && c <= epsi) return m
		c < 0 ? (h = m - 1) : (l = m + 1)
	}
	return neg ? ~l : l
}

cross = function (ax, ay, bx, by, cx, cy, dx, dy, co) {
	let abc = (ax - cx) * (by - cy) - (ay - cy) * (bx - cx)
	let abd = (ax - dx) * (by - dy) - (ay - dy) * (bx - dx)
	let cda = (cx - ax) * (dy - ay) - (cy - ay) * (dx - ax)
	let cdb = abc - abd + cda
	if (abc * abd >= 0 || cda * cdb >= 0) {
		if (!co || (abc && abd)) return [null, null, abc, abd, cda, cdb]
		return cda == 0 ? [ax, ay, abc, abd, cda, cdb] : [bx, by, abc, abd, cda, cdb]
	}
	let t = cda / (abd - abc)
	return [ax + t * (bx - ax), ay + t * (by - ay), abc, abd, cda, cdb]
}
area = function (s) {
	let [x0, y0] = (s = s.values?.() ?? s).next().value
	let [xx, yy] = [x0, y0]
	let a = 0
	for (let [x, y] of s) (a += (xx - x) * (yy + y)), (xx = x), (yy = y)
	return (a + (xx - x0) * (yy + y0)) / 2
}
