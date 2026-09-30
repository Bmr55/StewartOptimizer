export function installTooltips(document, window) {
    const infoPopup = document.createElement('div');
    infoPopup.className = 'info-popup';
    infoPopup.setAttribute('role', 'dialog');
    infoPopup.setAttribute('aria-label', 'More information');
    infoPopup.setAttribute('aria-hidden', 'true');
    document.body.appendChild(infoPopup);
    let activeInfoButton = null;
    // Scroll events are dispatched at the next rendering opportunity, so a click
    // that follows a scroll-position change in the same frame (scrollIntoView
    // then click, or a tap while smooth scrolling settles) would open the popup
    // and have the deferred scroll event close it one frame later. Scroll events
    // are ignored until the frame after the popup was shown.
    let shownFrame = 0;
    let ignoreScroll = false;
    const nextFrame = callback => typeof window.requestAnimationFrame === 'function'
        ? window.requestAnimationFrame(callback) : setTimeout(callback, 0);
    for (const button of document.querySelectorAll?.('.info-button') ?? []) button.setAttribute('aria-expanded', 'false');

    function hideInfoPopup() {
        if (!infoPopup.classList.contains('visible')) return;
        infoPopup.classList.remove('visible');
        infoPopup.setAttribute('aria-hidden', 'true');
        infoPopup.textContent = '';
        activeInfoButton?.setAttribute('aria-expanded', 'false');
        activeInfoButton = null;
    }

    function showInfoPopup(button) {
        const message = button?.dataset?.info;
        if (!message) return;
        activeInfoButton?.setAttribute('aria-expanded', 'false');
        infoPopup.textContent = message;
        infoPopup.classList.add('visible');
        infoPopup.setAttribute('aria-hidden', 'false');
        const rect = button.getBoundingClientRect();
        // clientWidth excludes a classic scrollbar, which innerWidth includes.
        const viewportWidth = document.documentElement?.clientWidth || window.innerWidth;
        const scrollX = window.scrollX || window.pageXOffset;
        const scrollY = window.scrollY || window.pageYOffset;
        const popupRect = infoPopup.getBoundingClientRect();
        const top = scrollY + rect.bottom + 8;
        let left = scrollX + rect.left + rect.width / 2 - popupRect.width / 2;
        const minLeft = scrollX + 8;
        const maxLeft = scrollX + viewportWidth - popupRect.width - 8;
        if (left < minLeft) left = minLeft;
        if (left > maxLeft) left = Math.max(minLeft, maxLeft);
        infoPopup.style.top = `${top}px`;
        infoPopup.style.left = `${left}px`;
        activeInfoButton = button;
        button.setAttribute('aria-expanded', 'true');
        const frame = ++shownFrame;
        ignoreScroll = true;
        nextFrame(() => { if (frame === shownFrame) ignoreScroll = false; });
    }

    document.addEventListener('click', (event) => {
        const button = event.target.closest('.info-button');
        if (button) {
            event.preventDefault();
            event.stopPropagation();
            if (activeInfoButton === button && infoPopup.classList.contains('visible')) {
                hideInfoPopup();
            } else {
                showInfoPopup(button);
            }
            return;
        }
        if (!infoPopup.contains(event.target)) {
            hideInfoPopup();
        }
    });

    document.addEventListener('keydown', (event) => {
        if (event.key === 'Escape') {
            hideInfoPopup();
        }
    });

    window.addEventListener('scroll', () => { if (!ignoreScroll) hideInfoPopup(); }, true);
    window.addEventListener('resize', hideInfoPopup);
}
